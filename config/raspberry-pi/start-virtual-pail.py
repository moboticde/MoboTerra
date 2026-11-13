import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import yaml

parser = argparse.ArgumentParser(description='Four-wheel-module virtual PAIL joystick program.')
parser.add_argument('--interface', required=True)
parser.add_argument('--profile', type=Path, default=Path('/home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/platform-virtual-pail.yaml'))
parser.add_argument('--rviz-config', type=Path, default=Path('/home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/virtual.rviz'))
parser.add_argument('--display', default=__import__('os').environ.get('DISPLAY', ':0'))
parser.add_argument('--xauthority', type=Path, default=Path(os.environ.get('XAUTHORITY', '/run/user/1000/gdm/Xauthority')))
parser.add_argument('--operator-source', type=Path, default=Path('/home/mobotic/Downloads/MoboTerra-main/src/mobotic_manual_control/mobotic_manual_control/remdevice.py'))
parser.add_argument('--runtime-image')
parser.add_argument('--reuse-rviz', action='store_true')
parser.add_argument('--no-rviz', action='store_true')
args = parser.parse_args()
virtual_profile = yaml.safe_load(args.profile.read_text())
if [module['name'] for module in virtual_profile.get('modules', [])] != [
        'front_left', 'front_right', 'rear_left', 'rear_right']:
    raise SystemExit('This virtual joystick program requires all four wheel modules.')
if __import__('os').geteuid() != 0:
    raise SystemExit('Run this launcher with sudo python3.')
if subprocess.run(['systemctl', 'is-active', '--quiet', 'moboterra-supervisor.service']).returncode == 0:
    raise SystemExit('The physical robot service is active. Stop it before launching this virtual joystick test.')
physical = subprocess.run(['docker', 'inspect', '--format', '{{.State.Running}}', 'moboterra-robot'], capture_output=True, text=True)
if physical.returncode == 0 and physical.stdout.strip() == 'true':
    raise SystemExit('The physical moboterra-robot container is running. Stop it before this virtual joystick test.')
for path in ([args.profile] if args.no_rviz else [args.profile, args.rviz_config, args.xauthority]):
    if not path.is_file():
        raise SystemExit(f'Missing file: {path}')
source_root = args.operator_source.parents[2]
manifest = json.loads(Path('/opt/moboterra/software/release.json').read_text())
if not args.runtime_image:
    commit = subprocess.run(['git', '-C', str(source_root.parent), 'rev-parse', '--short=12', 'HEAD'], capture_output=True, text=True)
    if commit.returncode == 0:
        images = subprocess.run(['docker', 'image', 'ls', '--filter',
            f'reference=moboterra:{commit.stdout.strip()}-*-jazzy-*',
            '--format', '{{.Repository}}:{{.Tag}}'], capture_output=True, text=True, check=True)
        candidates = sorted(images.stdout.splitlines(), reverse=True)
        if candidates:
            args.runtime_image = candidates[0]
runtime_image = args.runtime_image or manifest['runtime_image']
subprocess.run(['docker', 'image', 'inspect', runtime_image], check=True, stdout=subprocess.DEVNULL)
if subprocess.run(['docker', 'inspect', 'moboterra-virtual-rviz'], capture_output=True).returncode == 0:
    args.reuse_rviz = True
found = subprocess.run(['docker', 'inspect', '--format', '{{json .}}',
                        'moboterra-virtual'], capture_output=True, text=True)
existing = json.loads(found.stdout) if found.returncode == 0 else None
if existing:
    command = existing['Config'].get('Cmd') or []
    if 'stack_type:=virtual' not in command or 'robot_name:=moboterra_test' not in command:
        raise SystemExit('moboterra-virtual belongs to a different program; refusing to replace it.')
    mounted_profile = next((mount['Source'] for mount in existing.get('Mounts', [])
                            if mount['Destination'] == '/etc/moboterra/platform.yaml'), None)
    matches = (existing['Config']['Image'] == runtime_image
               and mounted_profile is not None
               and Path(mounted_profile).resolve() == args.profile.resolve()
               and f'operator_can_interface:={args.interface}' in command)
    environment = dict(value.split('=', 1) for value in existing['Config'].get('Env', []) if '=' in value)
    matches = (matches and environment.get('ROS_LOCALHOST_ONLY') == '0'
               and environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'SYSTEM_DEFAULT'
               and environment.get('FASTRTPS_DEFAULT_PROFILES_FILE') == '/etc/moboterra/virtual/virtual-fastdds.xml')
    if not matches:
        previous_logs = subprocess.run(['docker', 'logs', '--tail', '200', 'moboterra-virtual'],
                                       capture_output=True, text=True)
        log_path = args.profile.parent / ('virtual-before-update-' + time.strftime('%Y%m%d_%H%M%S') + '.log')
        log_path.write_text(previous_logs.stdout + previous_logs.stderr)
        if existing['State']['Running']:
            subprocess.run(['docker', 'stop', '--timeout', '10', 'moboterra-virtual'], check=True)
        subprocess.run(['docker', 'rm', 'moboterra-virtual'], check=True)
        existing = None
        print(f'Updating virtual container; previous logs saved to {log_path}.', flush=True)
link = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', 'dev', args.interface]))[0]
if 'UP' not in link['flags'] or link.get('linkinfo', {}).get('info_data', {}).get('bittiming', {}).get('bitrate') != 250000:
    raise SystemExit(f'{args.interface} must already be UP at the confirmed joystick bitrate 250000.')

common = ['docker', 'run', '--detach', '--init', '--network', 'host',
          '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges:true',
          '--env', 'ROS_DOMAIN_ID=142', '--env', 'ROS_LOCALHOST_ONLY=0',
          '--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT',
          '--env', 'FASTRTPS_DEFAULT_PROFILES_FILE=/etc/moboterra/virtual/virtual-fastdds.xml',
          '--env', 'FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
          '--tmpfs', '/var/log/ros:rw,nosuid,nodev,uid=10001,gid=10001,size=64m',
          '--tmpfs', '/tmp:rw,nosuid,nodev,uid=10001,gid=10001,size=128m',
          '--log-opt', 'max-size=10m', '--log-opt', 'max-file=2']
print(f'Using complete runtime image: {runtime_image}', flush=True)
if existing:
    if existing['State']['Running']:
        print('Reusing the running four-module virtual program.', flush=True)
    else:
        subprocess.run(['docker', 'start', 'moboterra-virtual'], check=True)
        print('Restarted the existing four-module virtual program.', flush=True)
else:
    subprocess.run(common + [
        '--name', 'moboterra-virtual',
        '--volume', f'{args.profile}:/etc/moboterra/platform.yaml:ro',
        '--volume', f'{args.profile.parent}:/etc/moboterra/virtual:ro',
        runtime_image, 'ros2', 'launch', 'mobotic_bringup', 'moboterra.launch.py',
        'stack_type:=virtual', 'robot_name:=moboterra_test',
        'platform_config:=/etc/moboterra/platform.yaml', 'start_manual_interface:=true',
        'start_scanners:=false', f'operator_can_interface:={args.interface}',
        'driver_parameters_file:=/etc/moboterra/virtual/virtual-driver-policy.yaml',
        'supervisor_parameters_file:=/etc/moboterra/virtual/virtual-supervisor-policy.yaml',
        'manual_parameters_file:=/etc/moboterra/virtual/virtual-manual-policy.yaml',
    ], check=True)
time.sleep(2)
state = subprocess.run(['docker', 'inspect', '--format', '{{json .State}}', 'moboterra-virtual'], capture_output=True, text=True)
if state.returncode or not json.loads(state.stdout).get('Running'):
    subprocess.run(['docker', 'logs', '--tail', '100', 'moboterra-virtual'])
    raise SystemExit('Virtual container exited during startup; logs are retained in moboterra-virtual.')
print('Virtual container running on ROS domain 142, namespace /moboterra_test.', flush=True)

check = r'''
import json, time
import rclpy
from sensor_msgs.msg import JointState
from mobotic_config.configuration import Configuration
from nav_msgs.msg import Odometry
from mobotic_interfaces.msg import OperatorState, VehicleModeState
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('virtual_ready_check')
samples = {}
expected_joints = set(Configuration('/etc/moboterra/platform.yaml').parameters('driver')['drives.names'])
if len(expected_joints) != 8:
    raise RuntimeError('The virtual joystick program requires eight joints for four wheel modules.')
handles = [node.create_subscription(kind, topic, lambda msg, key=key: samples.update({key: msg}), qos_profile_sensor_data)
    for kind, topic, key in (
        (JointState, '/moboterra_test/joint_states', 'joints'),
        (Odometry, '/moboterra_test/vehicle/odom', 'odom'),
        (OperatorState, '/moboterra_test/operator/state', 'operator'),
        (VehicleModeState, '/moboterra_test/vehicle/mode', 'mode'))]
deadline = time.monotonic() + 40
ready = False
feedback_ready = False
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if len(samples) == 4:
        mode = samples['mode']
        feedback_ready = (len(samples['joints'].name) == len(expected_joints)
                          and set(samples['joints'].name) == expected_joints)
        ready = (feedback_ready and mode.readiness_known and mode.ready_to_drive
                 and mode.drives_enabled and not mode.error_active and not mode.safety_stop_active)
        if ready:
            break
mode = samples.get('mode')
operator = samples.get('operator')
print(json.dumps({'ready': ready, 'feedback_ready': feedback_ready, 'received': sorted(samples),
    'joint_count': len(samples['joints'].name) if 'joints' in samples else 0,
    'operator_connected': operator.connected if operator else None,
    'operator_status': operator.status_message if operator else None,
    'motion_permitted': mode.motion_permitted if mode else None,
    'mode': mode.current_mode if mode else None,
    'status_message': mode.status_message if mode else None,
    'reason_codes': list(mode.reason_codes) if mode else None}, indent=2), flush=True)
node.destroy_node()
rclpy.shutdown()
'''
result = subprocess.run(['docker', 'exec', 'moboterra-virtual', '/usr/local/bin/moboterra-entrypoint',
                'python3', '-c', check])
if result.returncode:
    subprocess.run(['docker', 'logs', '--tail', '100', 'moboterra-virtual'])
    raise SystemExit('Virtual startup check failed; container logs were retained.')
if args.no_rviz:
    raise SystemExit(0)

import runpy
os.environ['DISPLAY'] = args.display
os.environ['XAUTHORITY'] = str(args.xauthority)
os.environ['MOBOTERRA_VIRTUAL_GUI_IMAGE'] = runtime_image.replace('moboterra:', 'moboterra-gui:', 1)
os.environ['MOBOTERRA_VIRTUAL_RVIZ_CONFIG'] = str(args.rviz_config)
runpy.run_path('/home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/open-virtual-rviz.py', run_name='__main__')
