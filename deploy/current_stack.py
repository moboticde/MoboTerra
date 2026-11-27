#!/usr/bin/env python3
"""Install the complete customer Docker stack, or prepare its host network and CAN."""
import argparse
from contextlib import contextmanager
import datetime
import hashlib
from ipaddress import IPv4Address, IPv4Network
import re
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SERVICE = 'moboterra-current-stack.service'
LEGACY_SERVICE = 'moboterra-supervisor.service'
RESTART_SOCKET_UNIT = 'moboterra-joystick-restart.socket'
RESTART_SERVICE_UNIT = 'moboterra-joystick-restart@.service'
RESTART_LISTENER_UNIT = 'moboterra-joystick-listener.service'
RESTART_SOCKET_DIR = '/run/moboterra-restart'
ENV_FILE = ROOT / 'runtime/docker-stack.env'
ROBOT_NETWORK = IPv4Network('192.168.0.0/16')
SCANNER_SUFFIXES = {'front_left': 5, 'rear_right': 6}
CAN_COMMAND_TIMEOUT = 5
sys.path[:0] = [str(ROOT / 'src/mobotic_config'), str(ROOT / 'src/mobotic_kinematics')]


def run(command, **kwargs):
    return subprocess.run(command, check=True, **kwargs)


def configuration(profile='platform.yaml'):
    from mobotic_config.configuration import COMPONENT_FILES, Configuration, ConfigurationError
    directory = ROOT / 'src/mobotic_config/config'
    try:
        return Configuration(directory / profile, overrides={
            component: directory / filename for component, filename in COMPONENT_FILES.items()})
    except ConfigurationError as error:
        raise ConfigurationError(f'{directory / profile}: {error}') from error


def can_plan(platform):
    """Shared drive/battery CAN remains one transmitting link, without renaming."""
    result = {}
    for role in ('drives', 'battery'):
        settings = platform['can'][role]
        interface = settings['interface']
        selected = (settings['bitrate'], bool(settings.get('listen_only', False)))
        if role == 'drives' and selected[1]:
            raise ValueError('The drive CAN bus must allow transmission')
        if interface in result and result[interface] != selected:
            raise ValueError(f'Conflicting CAN settings for shared interface {interface}')
        result[interface] = selected
    return result


class CanLinkError(ValueError):
    """The physical CAN link failed its startup checks."""


def validate_link(info, interface, bitrate, listen_only):
    if info.get('linkinfo', {}).get('info_kind') != 'can':
        raise ValueError(f'{interface} is not a physical SocketCAN interface')
    data = info['linkinfo']['info_data']
    if 'UP' not in info['flags']:
        raise CanLinkError(f'{interface} is not UP')
    if data.get('bittiming', {}).get('bitrate') != bitrate:
        raise CanLinkError(f'{interface} must use {bitrate} bit/s')
    if ('LISTEN-ONLY' in data.get('ctrlmode', [])) != listen_only:
        raise CanLinkError(f'{interface} has the wrong listen-only mode')
    if data.get('state') == 'BUS-OFF':
        raise CanLinkError(f'{interface} is BUS-OFF')


@contextmanager
def stack_stopped():
    import fcntl
    (ROOT / 'runtime').mkdir(exist_ok=True)
    with (ROOT / 'runtime/native-stack.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('Stop the current native/Docker stack before installing or preparing CAN') from error
        yield


def legacy_stopped():
    result = subprocess.run(['systemctl', 'is-active', '--quiet', LEGACY_SERVICE])
    if result.returncode == 0:
        raise RuntimeError(f'Stop the older deployment first: sudo systemctl stop {LEGACY_SERVICE}')


def is_robot_host_address(address):
    return address in ROBOT_NETWORK and address.packed[-1] not in (0, 255, *SCANNER_SUFFIXES.values())


def robot_ethernet_address(host_ip=None):
    interfaces = json.loads(run(['ip', '-json', '-details', '-4', 'address', 'show', 'up'],
                                capture_output=True, text=True).stdout)
    matches = []
    for info in interfaces:
        interface = info['ifname']
        sysfs = Path('/sys/class/net') / interface
        if (info.get('link_type') != 'ether' or 'LOWER_UP' not in info['flags']
                or not (sysfs / 'device').exists() or (sysfs / 'phy80211').exists()
                or (sysfs / 'wireless').exists()):
            continue
        for address in info.get('addr_info', []):
            local = address.get('local')
            if (local and is_robot_host_address(IPv4Address(local))
                    and (host_ip is None or local == host_ip)):
                matches.append((interface, local))
    if len(matches) > 1:
        candidates = ', '.join(f'{interface}={address}' for interface, address in matches)
        raise RuntimeError(f'Ambiguous robot Ethernet addresses: {candidates}. '
                           'Assign one robot address or select it with MOBOTERRA_ROBOT_HOST_IP.')
    return matches[0] if matches else None


def prepare_can_link(interface, bitrate, listen_only, deadline):
    show = ['ip', '-json', '-details', 'link', 'show', 'dev', interface]
    while True:
        probe = subprocess.run(show, capture_output=True, text=True, timeout=CAN_COMMAND_TIMEOUT)
        if probe.returncode == 0:
            info = json.loads(probe.stdout)[0]
            break
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f'Missing CAN interface: {interface}')
        time.sleep(min(1, remaining))

    if info.get('linkinfo', {}).get('info_kind') != 'can':
        raise ValueError(f'{interface} is not a physical SocketCAN interface')
    set_link = ['ip', 'link', 'set', 'dev', interface]
    cycle_at_start = interface == 'can0'
    if cycle_at_start:
        run([*set_link, 'down'], timeout=CAN_COMMAND_TIMEOUT)
    if cycle_at_start or 'UP' not in info['flags']:
        configure = [*set_link, 'type', 'can', 'bitrate', str(bitrate)]
        if not cycle_at_start:
            configure.extend(['listen-only', 'on' if listen_only else 'off', 'restart-ms', '0'])
        run(configure, timeout=CAN_COMMAND_TIMEOUT)
        run([*set_link, 'up'], timeout=CAN_COMMAND_TIMEOUT)
        if cycle_at_start:
            run(['ip', '-details', '-statistics', 'link', 'show', interface], timeout=CAN_COMMAND_TIMEOUT)
        info = json.loads(run(show, capture_output=True, text=True, timeout=CAN_COMMAND_TIMEOUT).stdout)[0]
    validate_link(info, interface, bitrate, listen_only)


def prepare_can(plan, deadline):
    """Cycle can0 at startup without USB resets or recovery retries."""
    for interface, (bitrate, listen_only) in sorted(plan.items()):
        prepare_can_link(interface, bitrate, listen_only, deadline)


def prepare_stack(wait_seconds=60):
    """Select the scanner network and prepare CAN before launching Docker."""
    import yaml
    from mobotic_config.configuration import validate_platform

    override = os.environ.get('MOBOTERRA_ROBOT_HOST_IP') or None
    if override and not is_robot_host_address(IPv4Address(override)):
        raise ValueError('MOBOTERRA_ROBOT_HOST_IP must be a usable 192.168.x.x scanner host address')

    legacy_stopped()
    with stack_stopped():
        deadline = time.monotonic() + wait_seconds
        selected = robot_ethernet_address(override)
        while selected is None:
            if time.monotonic() >= deadline:
                expected = override or 'a usable 192.168.x.x host address'
                raise RuntimeError(
                    f'No robot address on a connected wired Ethernet interface. Expected {expected}.')
            time.sleep(1)
            selected = robot_ethernet_address(override)
        interface, host_ip = selected
        subnet = host_ip.rsplit('.', 1)[0]
        platform = configuration().platform
        platform['scanners']['host_ip'] = host_ip
        for scanner in platform['scanners']['units']:
            scanner['sensor_ip'] = f'{subnet}.{SCANNER_SUFFIXES[scanner["name"]]}'
        validate_platform(platform)
        prepare_can(can_plan(platform), deadline)
        if robot_ethernet_address(override) != selected:
            raise RuntimeError('Robot Ethernet address changed during startup. Check the connection and restart.')
        atomic_text(ROOT / 'runtime/platform.yaml', yaml.safe_dump(platform, sort_keys=False))
        print(f'Scanner network selected on {interface} (host {host_ip}, '
              f'front left {subnet}.5, rear right {subnet}.6)', flush=True)


def systemd_quote(value):
    # systemd expands specifiers even inside quotes. No shell executes these paths.
    if any(character in str(value) for character in '\n\r\0'):
        raise ValueError('Paths must not contain newlines or NUL')
    value = str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%').replace('$', '$$')
    return '"' + value + '"'


def service_text(root, docker):
    compose = ' '.join(systemd_quote(value) for value in (
        docker, 'compose', '--env-file', root / 'runtime/docker-stack.env',
        '--file', root / 'deploy/compose.current-stack.yaml'))
    return f'''[Unit]
Description=MoboTerra current four-miControl ROS 2 stack in Docker
Requires=docker.service {RESTART_SOCKET_UNIT}
Wants=network-online.target {RESTART_LISTENER_UNIT}
After=docker.service network-online.target {RESTART_SOCKET_UNIT}
PartOf=docker.service
Conflicts={LEGACY_SERVICE}

[Service]
Type=simple
ExecStartPre=/usr/bin/python3 {systemd_quote(root / 'deploy/current_stack.py')} prepare
ExecStart={compose} up --no-build --abort-on-container-exit --exit-code-from robot
ExecStop={compose} stop --timeout 45
Restart=no
TimeoutStartSec=300
TimeoutStopSec=60

[Install]
WantedBy=multi-user.target
'''


def restart_units(root, uid, gid):
    return {
        RESTART_LISTENER_UNIT: f'''[Unit]
Description=Listen for the MoboTerra joystick restart gesture on host CAN
Requires={RESTART_SOCKET_UNIT}
After={RESTART_SOCKET_UNIT}

[Service]
Type=simple
User={uid}
Group={gid}
NoNewPrivileges=yes
RestrictAddressFamilies=AF_CAN AF_UNIX
ExecStart=/usr/bin/python3 -B {systemd_quote(root / 'deploy/host/joystick_restart.py')}
Restart=on-failure
RestartSec=2
''',
        RESTART_SOCKET_UNIT: f'''[Unit]
Description=MoboTerra joystick restart socket

[Socket]
ListenStream={RESTART_SOCKET_DIR}/restart.sock
SocketUser={uid}
SocketMode=0600
Accept=yes
MaxConnections=1
RemoveOnStop=yes
''',
        RESTART_SERVICE_UNIT: f'''[Unit]
Description=Restart the running MoboTerra stack from the joystick
CollectMode=inactive-or-failed

[Service]
Type=exec
StandardInput=socket
StandardOutput=journal
StandardError=journal
ExecStart={systemd_quote(root / 'deploy/host/moboterra_ctl.py')} restart --if-running
''',
    }


def env_text(values):
    for value in values.values():
        if any(character in str(value) for character in '\n\r\0'):
            raise ValueError('Environment values must not contain newlines or NUL')
    return ''.join(f"{key}='" + str(value).replace("'", "\\'") + "'\n" for key, value in values.items())



def read_environment(path=None):
    """Read our generated dotenv format without executing shell code."""
    values = {}
    for line in Path(path or ENV_FILE).read_text(encoding='utf-8').splitlines():
        if not line or line.startswith('#'):
            continue
        key, separator, value = line.partition('=')
        if not separator or not re.fullmatch(r'[A-Z][A-Z0-9_]*', key):
            raise ValueError('Invalid generated deployment environment')
        if len(value) < 2 or not (value.startswith("'") and value.endswith("'")):
            raise ValueError('Unexpected deployment environment encoding')
        values[key] = value[1:-1].replace("\\'", "'")
    return values


def runtime_identity(owner=None):
    import pwd
    account = owner or os.environ.get('SUDO_USER')
    if account and account != 'root':
        selected = pwd.getpwnam(account)
        return selected.pw_uid, selected.pw_gid
    source_owner = ROOT.stat()
    if source_owner.st_uid:
        return source_owner.st_uid, source_owner.st_gid
    # A root-owned installation does not require a native ROS/operator account.
    return 10001, 10001


def inspect_image(image):
    result = run(['docker', 'image', 'inspect', image], capture_output=True, text=True)
    record = json.loads(result.stdout)[0]
    if record.get('Os') != 'linux' or record.get('Architecture') not in ('amd64', 'arm64'):
        raise ValueError('The MoboTerra image must target Linux amd64 or arm64')
    labels = record.get('Config', {}).get('Labels') or {}
    if labels.get('com.mobotic.stack') != 'four-micontrol-bundled-romb-v1':
        raise ValueError('This image does not contain the complete current MoboTerra stack')
    daemon = run(['docker', 'info', '--format', '{{.OSType}}/{{.Architecture}}'],
                 capture_output=True, text=True).stdout.strip()
    daemon = daemon.replace('aarch64', 'arm64').replace('x86_64', 'amd64')
    if daemon != 'linux/' + record['Architecture']:
        raise ValueError('The image CPU architecture does not match the Docker host')
    return record['Id']


def verify_image(image):
    """Exercise the delivered image without physical networking or CAN."""
    run(['docker', 'run', '--rm', '--network', 'none', image, 'python3', '-c',
         'from mobotic_kinematics.romb.runtime import runtime_command; runtime_command([])'])
    run(['docker', 'run', '--rm', '--network', 'none', image,
         'bash', 'scripts/start_MoboTerra', '--check', 'start_rviz:=false'])
    run(['docker', 'run', '--rm', '--network', 'none', '--env', 'QT_QPA_PLATFORM=offscreen',
         image, 'rviz2', '--help'], stdout=subprocess.DEVNULL)
    run(['docker', 'run', '--rm', '--network', 'none', '--env', 'ROS_DOMAIN_ID=192',
         image, 'python3', '/opt/moboterra-checks/virtual_smoke.py'])


def acquire_image(selected=None, archive=None):
    if selected:
        if archive is not None:
            run(['docker', 'image', 'load', '--input', str(archive)])
        else:
            probe = subprocess.run(['docker', 'image', 'inspect', selected],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if probe.returncode:
                run(['docker', 'pull', selected])
        image = selected
    else:
        for profile in sorted((ROOT / 'src/mobotic_config/config').glob('platform-*.yaml')):
            configuration(profile.name)
        for name in ('agv_kinematics.deb', 'agv_kinematics_arm64.deb'):
            package = ROOT / 'deploy/vendor' / name
            expected = (package.with_suffix('.deb.sha256')).read_text().split()[0]
            if hashlib.sha256(package.read_bytes()).hexdigest() != expected:
                raise ValueError(f'The bundled ROMB package checksum does not match the delivery: {name}')
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d%H%M%S%f')
        image = 'moboterra-current:' + stamp
        revision = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', '--short=12', 'HEAD'],
                                  capture_output=True, text=True) if shutil.which('git') else None
        vcs_ref = revision.stdout.strip() if revision and revision.returncode == 0 else 'source-delivery'
        run(['docker', 'buildx', 'build', '--load', '--file', str(ROOT / 'deploy/Dockerfile'),
             '--target', 'current-stack', '--build-arg', 'VCS_REF=' + vcs_ref,
             '--build-arg', 'BUILD_DATE=' + stamp, '--tag', image, str(ROOT)])
    immutable = inspect_image(image)
    verify_image(immutable)
    return immutable


def atomic_text(path, content, mode=0o644):
    """Replace the destination itself, including an old symlink, without following it."""
    import tempfile
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix='.' + path.name + '-', dir=path.parent)
    temporary = Path(temporary)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as output:
            output.write(content)
        os.chmod(temporary, mode)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def register_ctl():
    destination = Path('/usr/local/bin/moboterra-ctl')
    destination.parent.mkdir(parents=True, exist_ok=True)
    source = ROOT / 'deploy/host/moboterra_ctl.py'
    source.chmod(source.stat().st_mode | 0o111)
    temporary = destination.with_name('.moboterra-ctl.new')
    temporary.unlink(missing_ok=True)
    try:
        temporary.symlink_to(source)
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)


def set_domain(domain):
    if domain is None:
        return
    if not 0 <= domain <= 232:
        raise ValueError('ROS domain ID must be between 0 and 232')
    path = ROOT / 'config/raspberry-pi/ros.env'
    original = path.stat()
    source = path.read_text()
    source, count = re.subn(r'^export ROS_DOMAIN_ID=.*$', 'export ROS_DOMAIN_ID=' + str(domain),
                           source, flags=re.MULTILINE)
    if count != 1:
        raise ValueError('Expected one ROS_DOMAIN_ID assignment in the project environment')
    atomic_text(path, source)
    os.chown(path, original.st_uid, original.st_gid)
    os.chmod(path, original.st_mode & 0o777)


def install(args):
    if os.geteuid() != 0:
        raise RuntimeError('Run bash install.sh; it obtains administrator access automatically')
    uid, gid = runtime_identity(args.owner)
    run(['docker', 'compose', 'version'])
    legacy_stopped()
    # Keep an active service's image selection and configuration untouched.
    if subprocess.run(['systemctl', 'is-active', '--quiet', SERVICE]).returncode == 0:
        raise RuntimeError('Stop the stack first: sudo moboterra-ctl stop')
    cfg = configuration()
    can_plan(cfg.platform)
    image = acquire_image(args.image, args.image_archive)
    if args.software_only and ENV_FILE.exists():
        register_ctl()
        print(f'Complete image {image} built and checked. Existing image selection and ROS domain retained; '
              'no robot service changed.')
        return
    runtime_dir = ROOT / 'runtime'
    for directory in (runtime_dir, runtime_dir / 'home', runtime_dir / 'ros'):
        directory.mkdir(exist_ok=True)
        os.chown(directory, uid, gid)
    with stack_stopped():
        os.chown(runtime_dir / 'native-stack.lock', uid, gid)
        legacy_stopped()
        set_domain(args.ros_domain_id)
        values = dict(MOBOTERRA_PROJECT_ROOT=ROOT, MOBOTERRA_CURRENT_IMAGE=image,
                      MOBOTERRA_UID=uid, MOBOTERRA_GID=gid,
                      MOBOTERRA_ROBOT_NAME=cfg.platform['robot_name'])
        pending_env = runtime_dir / 'docker-stack.pending.env'
        units = {}
        if not args.software_only:
            # Keep the legacy container parameter populated so older images
            # still reserve B1+B2. The real trigger socket stays on the host.
            values['MOBOTERRA_RESTART_SOCKET_DIR'] = runtime_dir
            values['MOBOTERRA_HOST_RESTART'] = 'true'
            units = {SERVICE: service_text(ROOT, shutil.which('docker')), **restart_units(ROOT, uid, gid)}
        pending_units = {runtime_dir / name: contents for name, contents in units.items()}
        pending_env.write_text(env_text(values))
        env = {key: value for key, value in os.environ.items() if key not in values}
        try:
            for path, contents in pending_units.items():
                path.write_text(contents)
            run(['docker', 'compose', '--env-file', str(pending_env), '--file',
                 str(ROOT / 'deploy/compose.current-stack.yaml'), 'config', '--quiet'], env=env)
            if pending_units:
                run(['systemd-analyze', 'verify', *map(str, pending_units)])
            if ENV_FILE.exists():
                atomic_text(runtime_dir / 'docker-stack.previous.env', ENV_FILE.read_text())
            atomic_text(ENV_FILE, pending_env.read_text())
            register_ctl()
            for name, contents in units.items():
                atomic_text(Path('/etc/systemd/system') / name, contents)
        finally:
            pending_env.unlink(missing_ok=True)
            for path in pending_units:
                path.unlink(missing_ok=True)
        if not args.software_only:
            load = subprocess.run(['systemctl', 'show', LEGACY_SERVICE, '--property=LoadState', '--value'],
                                  capture_output=True, text=True).stdout.strip()
            if load and load != 'not-found':
                run(['systemctl', 'disable', LEGACY_SERVICE])
            run(['systemctl', 'daemon-reload'])
            run(['systemctl', 'disable' if args.no_start else 'enable', SERVICE])
    if args.start:
        run(['systemctl', 'start', SERVICE])
    state = ('no robot service configured' if args.software_only else
             'service installed with boot startup disabled' if args.no_start else
             'service enabled for the next boot')
    next_step = ('Run the installer without --software-only to configure the robot service.' if args.software_only else
                 'Stack started; normal B2/readiness checks remain active.' if args.start else
                 'Robot stack remains stopped. Start explicitly with sudo moboterra-ctl start.')
    print(f'Installed complete MoboTerra image {image}; {state}.\n'
          'No native ROS installation or host ROMB runtime is required.\n' + next_step)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    installer = commands.add_parser('install', help='Install the complete container and current boot service')
    installer.add_argument('--owner', help='Optional runtime user; inferred automatically when omitted')
    installer.add_argument('--image', help='Optional prepared complete image; otherwise build the bundled source')
    installer.add_argument('--image-archive', type=Path, help='Load a local Docker archive containing --image')
    installer.add_argument('--ros-domain-id', type=int, choices=range(233), metavar='ID')
    mode = installer.add_mutually_exclusive_group()
    mode.add_argument('--no-start', action='store_true', help='Install with boot startup disabled; do not start')
    mode.add_argument('--software-only', action='store_true', help='Build/check image and install CLI without a robot service')
    mode.add_argument('--start', action='store_true', help='Enable boot startup and start the ROS stack now')
    commands.add_parser('prepare', help='Boot preflight: select the scanner profile and prepare CAN')
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('Run on the Ubuntu robot PC; physical SocketCAN requires Linux')
    if args.command == 'install' and args.image_archive is not None and not args.image:
        installer.error('--image-archive requires --image')
    try:
        if args.command == 'install':
            install(args)
        else:
            if os.geteuid() != 0:
                raise RuntimeError('CAN preparation requires root')
            prepare_stack()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        raise SystemExit(str(error)) from error


if __name__ == '__main__':
    main()
