#!/usr/bin/python3
"""Stage a published release and apply it during operator-approved maintenance."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import pwd
import re
import shlex
import subprocess
import sys
import time

CONFIG = Path('/etc/moboterra-fleet.json')
STATE = Path('/var/lib/moboterra-fleet')
SERVICE = 'moboterra-current-stack.service'
CONTAINER = 'moboterra-current-stack'
MAIN_REF = 'refs/moboterra-fleet/main'
APPROVAL_REF = 'refs/moboterra-fleet/approved'
COMMIT = re.compile(r'[0-9a-f]{40}')
DIGEST = re.compile(r'sha256:[0-9a-f]{64}')
DOCKER_PLATFORMS = {
    'linux/x86_64': 'amd64', 'linux/amd64': 'amd64',
    'linux/aarch64': 'arm64', 'linux/arm64': 'arm64',
}


def run(*args, capture=False, check=True, timeout=1800):
    return subprocess.run(args, check=check, text=True, timeout=timeout,
                          stdout=subprocess.PIPE if capture else None)


def read_json(path):
    return json.loads(path.read_text())


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as handle:
        json.dump(value, handle, indent=2)
        handle.write('\n')
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


class Fleet:
    def __init__(self, config, state=STATE):
        self.checkout = Path(config['checkout'])
        self.owner = config['owner']
        self.robot = config['robot']
        self.image_repository = config['image_repository']
        self.staged = state / 'staged.json'
        self.pending = state / 'pending.json'
        self.previous = state / 'previous.json'
        if not self.checkout.is_absolute() or not (self.checkout / '.git').is_dir():
            raise ValueError('checkout must be an absolute path to the installed Git clone')
        if self.checkout.stat().st_uid != pwd.getpwnam(self.owner).pw_uid or self.owner == 'root':
            raise ValueError('owner must be the non-root account that owns the checkout')
        if not re.fullmatch(r'MoboTerra_[1-3]', self.robot):
            raise ValueError('robot must be MoboTerra_1, MoboTerra_2 or MoboTerra_3')
        if not re.fullmatch(r'ghcr\.io/[a-z0-9_.-]+/[a-z0-9_.-]+', self.image_repository):
            raise ValueError('image_repository must be a lowercase GHCR repository without tag')

    def git(self, *args, **kwargs):
        return run('runuser', '-u', self.owner, '--', 'env', 'GIT_TERMINAL_PROMPT=0',
                   'git', '-C', str(self.checkout), *args, **kwargs)

    def revision(self):
        return self.git('rev-parse', 'HEAD', capture=True).stdout.strip()

    def validate_release(self, release):
        commit, image = release['commit'], release['image']
        repository, separator, digest = image.partition('@')
        if not (COMMIT.fullmatch(commit) and separator and
                repository == self.image_repository and DIGEST.fullmatch(digest)):
            raise ValueError('Invalid commit or pinned image in release record')
        return commit, image

    def stage(self):
        channel = f'refs/heads/fleet/{self.robot}'
        remote = self.git('ls-remote', '--exit-code', '--refs', 'origin', channel,
                          capture=True, check=False)
        if remote.returncode == 2:
            self.staged.unlink(missing_ok=True)
            print(f'No approved release for {self.robot}', flush=True)
            return None
        remote.check_returncode()
        self.git('fetch', '--no-tags', 'origin', f'+refs/heads/main:{MAIN_REF}',
                 f'+{channel}:{APPROVAL_REF}')
        release = json.loads(self.git('show', f'{APPROVAL_REF}:release.json', capture=True).stdout)
        commit, image = self.validate_release(release)
        self.git('merge-base', '--is-ancestor', commit, MAIN_REF)
        daemon = run('docker', 'info', '--format', '{{.OSType}}/{{.Architecture}}',
                     capture=True).stdout.strip()
        architecture = DOCKER_PLATFORMS.get(daemon)
        if architecture is None:
            raise ValueError(f'Unsupported Docker host platform: {daemon}')
        if run('docker', 'image', 'inspect', '--format', '{{.Id}}', image,
               capture=True, check=False).returncode:
            run('docker', 'pull', '--platform', f'linux/{architecture}', image)
        metadata = json.loads(run('docker', 'image', 'inspect', '--format', '{{json .}}',
                                  image, capture=True).stdout)
        labels = metadata['Config'].get('Labels') or {}
        if (metadata['Os'], metadata['Architecture']) != ('linux', architecture) or (
                labels.get('org.opencontainers.image.revision') != commit or
                labels.get('com.mobotic.stack') != 'four-micontrol-bundled-romb-v1'):
            raise ValueError('Published image architecture, revision or stack label does not match')
        staged = {'commit': commit, 'image': image, 'id': metadata['Id']}
        write_json(self.staged, staged)
        print(f'Staged {commit} for {self.robot}', flush=True)
        return staged

    def installed_image(self):
        path = self.checkout / 'runtime/docker-stack.env'
        values = dict(line.split('=', 1) for line in path.read_text().splitlines()
                      if '=' in line and not line.startswith('#'))
        root, = shlex.split(values['MOBOTERRA_PROJECT_ROOT'])
        image, = shlex.split(values['MOBOTERRA_CURRENT_IMAGE'])
        if Path(root).resolve() != self.checkout.resolve() or not DIGEST.fullmatch(image):
            raise ValueError('The configured checkout and installed deployment do not match')
        return image

    def clean_checkout(self):
        if self.git('status', '--porcelain', capture=True).stdout:
            raise RuntimeError('Checkout has local changes. Preserve and review them before updating.')

    def stop(self):
        run('/usr/local/bin/moboterra-ctl', 'stop', timeout=90)
        active = run('systemctl', 'show', '-p', 'ActiveState', '--value', SERVICE, capture=True).stdout.strip()
        if active != 'inactive':
            raise RuntimeError(f'Expected an inactive MoboTerra service, got {active}')
        running = run('docker', 'container', 'ls', '--quiet', '--filter',
                      f'name=^/{CONTAINER}$', capture=True).stdout.strip()
        if running:
            raise RuntimeError('MoboTerra container is still running')

    def select(self, commit, image):
        # A reboot during cutover must not launch an image against mixed host files.
        run('systemctl', 'disable', SERVICE)
        self.git('switch', '--detach', commit)
        run('/bin/bash', str(self.checkout / 'install.sh'), '--owner', self.owner,
            '--image', image, '--no-start')

    def restore_boot_setting(self, enabled):
        if enabled:
            run('systemctl', 'enable', SERVICE)

    def wait_for_health(self, image_id, timeout=240):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            service_state = run('systemctl', 'show', '-p', 'ActiveState', '--value',
                                SERVICE, capture=True).stdout.strip()
            if service_state == 'failed':
                break
            result = run('docker', 'container', 'inspect', '--format',
                         '{{json .State}} {{.Image}}', CONTAINER, capture=True, check=False)
            if result.returncode == 0:
                state, found_image = result.stdout.strip().rsplit(' ', 1)
                state = json.loads(state)
                if found_image == image_id and state['Running']:
                    health = (state.get('Health') or {}).get('Status')
                    if health == 'healthy' and service_state == 'active':
                        return
                    if health == 'unhealthy':
                        break
            time.sleep(3)
        raise RuntimeError('The requested image did not become healthy. See the service and container logs.')

    def apply(self, expected_commit):
        if self.pending.exists():
            raise RuntimeError('An update was interrupted. Run rollback before applying another release.')
        self.clean_checkout()
        staged = self.stage()
        if staged is None or staged['commit'] != expected_commit:
            raise RuntimeError('The approved release does not match the commit cleared for maintenance.')
        old_commit = self.revision()
        old_image = self.installed_image()
        if (old_commit, old_image) == (staged['commit'], staged['id']):
            print('Release already installed.', flush=True)
            return
        self.git('merge-base', '--is-ancestor', old_commit, expected_commit)
        run('docker', 'image', 'inspect', '--format', '{{.Id}}', old_image, capture=True)
        enabled = run('systemctl', 'is-enabled', SERVICE, capture=True, check=False).stdout.strip()
        if enabled not in ('enabled', 'disabled'):
            raise RuntimeError(f'Unsupported stack boot setting: {enabled}')
        write_json(self.pending, {'commit': old_commit, 'image': old_image, 'enabled': enabled == 'enabled'})
        self.stop()
        try:
            self.select(expected_commit, staged['image'])
            run('/usr/local/bin/moboterra-ctl', 'start', timeout=120)
            self.wait_for_health(staged['id'])
            self.restore_boot_setting(enabled == 'enabled')
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            print(f'Update failed: {error}. Restoring previous release with ROS stopped.', file=sys.stderr)
            self.restore(self.pending)
            raise RuntimeError('Update failed. Previous release restored and stopped.') from error
        self.pending.replace(self.previous)
        print(f'Running healthy release {expected_commit}', flush=True)

    def restore(self, record):
        previous = read_json(record)
        self.clean_checkout()
        run('docker', 'image', 'inspect', '--format', '{{.Id}}', previous['image'], capture=True)
        self.stop()
        self.select(previous['commit'], previous['image'])
        self.restore_boot_setting(previous['enabled'])
        record.unlink()
        print(f"Restored {previous['commit']}. ROS is stopped.", flush=True)

    def rollback(self):
        record = self.pending if self.pending.exists() else self.previous
        if not record.exists():
            raise RuntimeError('No previous fleet release is recorded.')
        self.restore(record)

    def status(self):
        print(f'Robot: {self.robot}\nCheckout: {self.revision()}\nImage: {self.installed_image()}')
        for path in (self.staged, self.pending, self.previous):
            if path.exists():
                print(f'{path.stem}: {path.read_text().strip()}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('stage', 'status'):
        commands.add_parser(name)
    apply = commands.add_parser('apply')
    apply.add_argument('commit')
    rollback = commands.add_parser('rollback')
    for command in (apply, rollback):
        command.add_argument('--maintenance', action='store_true', required=True,
                             help='Confirm robot is parked, drives disabled, and cleared for maintenance')
    args = parser.parse_args()
    try:
        if os.geteuid() != 0:
            raise RuntimeError('Run through sudo.')
        state = STATE
        state.mkdir(mode=0o700, exist_ok=True)
        with (state / 'lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fleet = Fleet(read_json(CONFIG))
            if args.command == 'apply':
                fleet.apply(args.commit)
            else:
                getattr(fleet, args.command)()
    except (OSError, KeyError, TypeError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f'moboterra-fleet: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
