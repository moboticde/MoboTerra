#!/usr/bin/env python3
"""Customer controls for the complete current MoboTerra Docker stack."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import current_stack as deployment

SERVICE = deployment.SERVICE
CONTAINER = 'moboterra-current-stack'


def run(command, *, check=False):
    return subprocess.run(command, text=True, check=check)


def require_root():
    if os.geteuid() != 0:
        raise RuntimeError('Run this command with sudo')


def environment():
    if not deployment.ENV_FILE.is_file():
        raise RuntimeError('MoboTerra is not installed. Run bash install.sh from the delivery directory.')
    values = deployment.read_environment()
    required = {'MOBOTERRA_CURRENT_IMAGE', 'MOBOTERRA_UID', 'MOBOTERRA_GID', 'MOBOTERRA_ROBOT_NAME'}
    if not required <= values.keys() or not values['MOBOTERRA_UID'].isdigit() or not values['MOBOTERRA_GID'].isdigit():
        raise ValueError('Incomplete deployment environment; rerun bash install.sh from the delivery directory')
    return values


def show_status(_args):
    values = environment()
    print('Image: ' + values['MOBOTERRA_CURRENT_IMAGE'], flush=True)
    result = run(['systemctl', '--no-pager', '--full', 'status', SERVICE])
    run(['docker', 'inspect', '--format',
         'container={{.State.Status}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} image={{.Config.Image}}',
         CONTAINER])
    print('Service/container status does not establish drive readiness; inspect vehicle/mode.', flush=True)
    return result.returncode


def show_can(_args):
    for interface in deployment.can_plan(deployment.configuration().platform):
        result = run(['ip', '-details', '-statistics', 'link', 'show', 'dev', interface])
        if result.returncode:
            return result.returncode
    return 0


def service_action(args):
    require_root()
    environment()
    command = ['systemctl', args.action]
    if getattr(args, 'now', False): command.append('--now')
    command.append(SERVICE)
    run(command, check=True)
    return 0


def show_logs(args):
    if args.host:
        command = ['journalctl', '-u', SERVICE, '--no-pager', '-n', str(args.lines)]
        if args.follow: command.append('--follow')
    else:
        command = ['docker', 'logs', '--tail', str(args.lines)]
        if args.follow: command.append('--follow')
        command.append(CONTAINER)
    return run(command).returncode


def ros_command(args):
    if not args.arguments:
        raise ValueError('Supply a ros2 command, for example: moboterra-ctl ros topic list')
    return run(['docker', 'exec', CONTAINER, 'bash', '-c',
                'source scripts/setup_ros.bash && exec ros2 "$@"', '--', *args.arguments]).returncode


def shell(_args):
    return run(['docker', 'exec', '-it', CONTAINER, 'bash', '-c',
                'source scripts/setup_ros.bash && exec bash']).returncode


def isolated_command(values, command):
    root = deployment.ROOT
    return ['docker', 'run', '--rm', '--network', 'none', '--read-only',
            '--user', values['MOBOTERRA_UID'] + ':' + values['MOBOTERRA_GID'],
            '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges:true',
            '--tmpfs', '/tmp:rw,mode=1777',
            '--tmpfs', '/opt/moboterra-project/runtime:rw,uid=' + values['MOBOTERRA_UID'] + ',gid=' + values['MOBOTERRA_GID'],
            '--mount', f'type=bind,src={root / "src/mobotic_config/config"},dst=/opt/moboterra/share/mobotic_config/config,readonly',
            '--mount', f'type=bind,src={root / "config/raspberry-pi/ros.env"},dst=/opt/moboterra-project/config/raspberry-pi/ros.env,readonly',
            values['MOBOTERRA_CURRENT_IMAGE'], *command]


def check(_args):
    values = environment()
    return run(isolated_command(values, ['bash', 'scripts/start_MoboTerra',
                                        '--check', 'start_rviz:=false'])).returncode


def rviz(_args):
    """Run the image's RViz as the desktop user; no native ROS or xhost changes."""
    import pwd
    values = environment()
    display = os.environ.get('DISPLAY')
    if not display or not Path('/tmp/.X11-unix').is_dir():
        raise RuntimeError('Run moboterra-ctl rviz from the robot graphical desktop with X11/XWayland available')
    uid = int(os.environ.get('SUDO_UID', os.getuid()))
    account = pwd.getpwuid(uid)
    authority = Path(os.environ.get('XAUTHORITY') or Path(account.pw_dir) / '.Xauthority')
    if not authority.is_file():
        raise RuntimeError('The desktop Xauthority file is unavailable; run from the logged-in desktop session')
    root = deployment.ROOT
    namespace = values['MOBOTERRA_ROBOT_NAME']
    prefix = '/' + namespace.strip('/') if namespace else ''
    docker = ['docker']
    if os.geteuid() != 0 and subprocess.run(['docker', 'info'], stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL).returncode:
        if not shutil.which('sudo'):
            raise RuntimeError('Docker access requires sudo on this computer')
        docker = ['sudo', 'docker']
    command = [*docker, 'run', '--rm', '--network', 'host', '--ipc', 'host', '--pid', 'host', '--read-only',
        '--user', f'{uid}:{account.pw_gid}', '--cap-drop', 'ALL',
        '--security-opt', 'no-new-privileges:true', '--env', 'DISPLAY=' + display,
        '--env', 'XAUTHORITY=/tmp/moboterra.xauthority', '--env', 'HOME=/tmp',
        '--env', 'LIBGL_ALWAYS_SOFTWARE=1', '--env', 'QT_X11_NO_MITSHM=1', '--tmpfs', '/tmp:rw,mode=1777',
        '--tmpfs', f'/opt/moboterra-project/runtime:rw,uid={uid},gid={account.pw_gid}',
        '--mount', 'type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly',
        '--mount', f'type=bind,src={authority},dst=/tmp/moboterra.xauthority,readonly',
        '--mount', f'type=bind,src={root / "config/raspberry-pi/ros.env"},dst=/opt/moboterra-project/config/raspberry-pi/ros.env,readonly',
        '--mount', f'type=bind,src={root / "config/raspberry-pi/live.rviz"},dst=/opt/moboterra-project/config/raspberry-pi/live.rviz,readonly',
        values['MOBOTERRA_CURRENT_IMAGE'], 'bash', '-c',
        'source scripts/setup_ros.bash && exec rviz2 -d config/raspberry-pi/live.rviz --ros-args "$@"',
        '--', '-r', '__ns:=' + prefix, '-r', '/tf:=' + prefix + '/tf',
        '-r', '/tf_static:=' + prefix + '/tf_static']
    return run(command).returncode


def rollback(args):
    require_root()
    current = environment()
    previous_path = deployment.ROOT / 'runtime/docker-stack.previous.env'
    previous = deployment.read_environment(previous_path)
    image = deployment.inspect_image(previous['MOBOTERRA_CURRENT_IMAGE'])
    run(['systemctl', 'stop', SERVICE], check=True)
    selected = dict(current, MOBOTERRA_CURRENT_IMAGE=image)
    # Test the previous image with the current configuration, with no hardware network.
    # Previous images may predate the launcher rename. Select the entry point
    # inside that image rather than assuming it matches this host checkout.
    preflight = (
        'for launcher in scripts/start_MoboTerra scripts/start_ros2_four_micontrol.sh; do '
        'if [[ -f "$launcher" ]]; then exec bash "$launcher" --check start_rviz:=false; fi; '
        'done; echo "MoboTerra launcher missing from rollback image" >&2; exit 1'
    )
    run(isolated_command(selected, ['bash', '-c', preflight]), check=True)
    deployment.atomic_text(deployment.ENV_FILE, deployment.env_text(selected))
    deployment.atomic_text(previous_path, deployment.env_text(current))
    print('Previous image selected. Current configuration was retained. Stack remains stopped.', flush=True)
    if args.start:
        run(['systemctl', 'start', SERVICE], check=True)
    return 0


def positive(value):
    selected = int(value)
    if selected < 1:
        raise argparse.ArgumentTypeError('Must be a positive integer')
    return selected


def main(argv=None):
    parser = argparse.ArgumentParser(prog='moboterra-ctl', description=__doc__)
    commands = parser.add_subparsers(dest='command')
    for name, function in [('status', show_status), ('can', show_can), ('check', check),
                           ('shell', shell), ('rviz', rviz)]:
        commands.add_parser(name).set_defaults(func=function)
    for action in ('start', 'stop', 'restart', 'enable', 'disable'):
        command = commands.add_parser(action)
        if action in ('enable', 'disable'):
            command.add_argument('--now', action='store_true', help='Also start/stop this session')
        command.set_defaults(func=service_action, action=action)
    logs = commands.add_parser('logs')
    logs.add_argument('--follow', action='store_true')
    logs.add_argument('--host', action='store_true', help='Host service journal instead of ROS container output')
    logs.add_argument('--lines', type=positive, default=100)
    logs.set_defaults(func=show_logs)
    ros = commands.add_parser('ros')
    ros.add_argument('arguments', nargs=argparse.REMAINDER)
    ros.set_defaults(func=ros_command)
    old = commands.add_parser('rollback')
    old.add_argument('--start', action='store_true', help='Start after the image rollback; otherwise leave stopped')
    old.set_defaults(func=rollback)
    args = parser.parse_args(argv)
    if not hasattr(args, 'func'):
        args = parser.parse_args(['status'])
    try:
        return args.func(args)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, str(error) + '\n')


if __name__ == '__main__':
    sys.exit(main())
