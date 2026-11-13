#!/usr/bin/env python3
"""Link the stopped managed deployment to project-owned settings. Never start it."""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SERVICE = 'moboterra-supervisor.service'
LINKS = {
    Path('/etc/moboterra'): PROJECT_ROOT / 'config/managed',
    Path('/opt/moboterra/current/config'): PROJECT_ROOT / 'config/managed/installed-defaults',
    Path('/opt/moboterra/current/compose.yaml'): PROJECT_ROOT / 'config/managed/compose.yaml',
    Path('/etc/systemd/system/moboterra-supervisor.service'): PROJECT_ROOT / 'config/systemd/moboterra-supervisor.service',
    Path('/etc/tmpfiles.d/moboterra.conf'): PROJECT_ROOT / 'config/systemd/moboterra-tmpfiles.conf',
}

# Manual-start policy is owned by this project. A settings reinstall must not
# silently restore a system service the operator explicitly removed.
POLICY_FILE = PROJECT_ROOT / 'config/startup-policy.json'
REGISTER_SERVICE = (json.loads(POLICY_FILE.read_text()).get('managed_systemd_service', True)
                    if POLICY_FILE.exists() else True)
if not REGISTER_SERVICE:
    LINKS.pop(Path('/etc/systemd/system/moboterra-supervisor.service'))


def validate_sources():
    manifest = json.loads((PROJECT_ROOT / 'config/managed/migration-sources.json').read_text())
    for source, target in LINKS.items():
        if not target.exists():
            raise RuntimeError(f'Project target is missing: {target}')
        if source.is_symlink() and source.resolve() == target.resolve():
            continue
        if source.is_symlink():
            raise RuntimeError(f'Unexpected existing link: {source} -> {source.readlink()}')
        if not source.exists():
            raise RuntimeError(f'Original configuration is missing: {source}')
        expected = manifest[str(source)]
        if source.is_dir():
            actual_names = {p.name for p in source.iterdir() if p.is_file()}
            if actual_names != set(expected):
                raise RuntimeError(f'Configuration file inventory changed: {source}')
        for relative, checksum in expected.items():
            path = source if relative == '.' else source / relative
            if hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
                raise RuntimeError(f'Original changed since staging; refusing to overwrite: {path}')
        # Except for the deliberately updated systemd unit, require identical
        # contents at the new location so moving cannot change robot policies.
        if source.name != 'moboterra-supervisor.service':
            for relative, checksum in expected.items():
                path = target if relative == '.' else target / relative
                if hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
                    raise RuntimeError(f'Staged settings differ from original: {path}')


def relocate_link(source, target, backup):
    """Move an original into the project, then install a link; recover on failure."""
    if source.is_symlink() and source.resolve() == target.resolve():
        return False
    backup.parent.mkdir(parents=True, exist_ok=True)
    if backup.exists() or backup.is_symlink():
        raise RuntimeError(f'Backup already exists: {backup}')
    shutil.move(str(source), str(backup))
    try:
        source.symlink_to(target, target_is_directory=target.is_dir())
    except BaseException:
        shutil.move(str(backup), str(source))
        raise
    return True


def restore_link(source, backup):
    if source.is_symlink():
        source.unlink()
    shutil.move(str(backup), str(source))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Read-only preflight; no root or CAN access required')
    args = parser.parse_args()
    validate_sources()
    if args.check:
        for source, target in LINKS.items():
            state = 'linked' if source.is_symlink() and source.resolve() == target.resolve() else 'pending sudo installation'
            print(f'{source} -> {target} [{state}]')
        return
    if os.geteuid() != 0:
        raise SystemExit(f'Administrative access required. Run: sudo python3 {Path(__file__).resolve()}')
    result = subprocess.run(['systemctl', 'is-active', '--quiet', SERVICE])
    if result.returncode not in (3, 4):
        raise SystemExit('Managed service must be stopped before migration; it will not be stopped automatically.')
    (PROJECT_ROOT / 'runtime').mkdir(exist_ok=True)
    with (PROJECT_ROOT / 'runtime/native-stack.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Stop the native stack before installing the system configuration links.')
        validate_sources()
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        backup_dir = PROJECT_ROOT / 'runtime' / ('system-settings-before-' + stamp)
        changed = []
        try:
            for index, (source, target) in enumerate(LINKS.items()):
                backup = backup_dir / str(index) / source.name
                if relocate_link(source, target, backup):
                    changed.append((source, backup))
            subprocess.run(['systemd-analyze', 'verify', str(PROJECT_ROOT / 'config/systemd/moboterra-supervisor.service')], check=True)
            subprocess.run(['systemctl', 'daemon-reload'], check=True)
        except BaseException:
            for source, backup in reversed(changed):
                restore_link(source, backup)
            subprocess.run(['systemctl', 'daemon-reload'], check=False)
            raise
        status = {'installed': True, 'links': {str(k): str(v) for k, v in LINKS.items()},
                  'backups': str(backup_dir), 'service_started': False,
                  'managed_systemd_service': bool(REGISTER_SERVICE)}
        (PROJECT_ROOT / 'runtime/system-settings-installation.json').write_text(json.dumps(status, indent=2) + '\n')
        print('System settings now resolve into the MoboTerra project. Service remains stopped.')
        print(f'Original root-owned files and historical backups: {backup_dir}')


if __name__ == '__main__':
    main()
