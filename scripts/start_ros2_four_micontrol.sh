#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
mkdir -p "$PROJECT_ROOT/runtime"
# Acquire shared stack lock before any checks or communication.
exec 9>"$PROJECT_ROOT/runtime/native-stack.lock"
if ! flock -n 9; then
  echo 'A MoboTerra native launch is already running; stop it before starting another.' >&2
  exit 1
fi
cd "$PROJECT_ROOT"
# Load the fixed runtime independently of overlays inherited from the terminal.
# Re-sourcing setup.bash does not reorder prefixes that are already present.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source "$PROJECT_ROOT/scripts/setup_ros.bash"
config_dir="$PROJECT_ROOT/config/raspberry-pi"
profile=$(python3 -c 'from mobotic_config.configuration import default_platform_path; print(default_platform_path())')
drive_interface=$(python3 -c 'from mobotic_config.configuration import Configuration; print(Configuration().platform["can"]["drives"]["interface"])')
ros2 run mobotic_config check_config >/dev/null
# Visualization flags are defaults; explicit ROS launch arguments override them.
start_rviz=true
start_description=true
start_odometry=true
rviz_config="$config_dir/live.rviz"
for argument in "$@"; do
  case "$argument" in
    start_rviz:=*) start_rviz="${argument#*:=}" ;;
    start_description:=*) start_description="${argument#*:=}" ;;
    start_odometry:=*) start_odometry="${argument#*:=}" ;;
    rviz_config:=*) rviz_config="${argument#*:=}" ;;
  esac
done
visualization_args=(--profile "$profile" --rviz-config "$rviz_config"
  --start-rviz "$start_rviz" --start-description "$start_description" --start-odometry "$start_odometry")
if [[ "${1:-}" != "--check" ]]; then
  visualization_args+=(--require-display)
fi
python3 "$PROJECT_ROOT/tools/pi/check_visualization.py" "${visualization_args[@]}"
if [[ "${1:-}" == "--check" ]]; then
  echo "Project configuration validated; no CAN access or robot startup performed."
  exit 0
fi
if systemctl is-active --quiet moboterra-supervisor.service; then
  echo 'Stop the managed stack before starting the native stack.' >&2
  exit 1
fi
python3 - <<'PY'
import json, os, pathlib, socket, subprocess
from ament_index_python.packages import get_package_prefix
from mobotic_config.configuration import Configuration
for package, executables in {
    'mobotic_driver': ('mobotic_driver', 'paced_socket_can_sender'),
    'mobotic_bringup': ('manual_drive_control',),
    'mobotic_kinematics': ('romb_kinematics', 'romb_kinematics_adapter'),
    'mobotic_manual_control': ('remdevice_control',),
}.items():
    prefix = pathlib.Path(get_package_prefix(package))
    for executable in executables:
        path = prefix / 'lib' / package / executable
        if not path.is_file() or not os.access(path, os.X_OK):
            raise SystemExit(f'Missing executable: {path}. No robot nodes started; repair the runtime overlay.')
node_executables = {
    'manual_drive_control', 'mobotic_driver', 'mobotic_kinematics', 'mobotic_odometry', 'mobotic_supervisor',
    'romb_kinematics', 'romb_kinematics_adapter', 'qemu-x86_64-static',
    'remdevice_control', 'manual_control', 'flexisoft_tcp_bridge', 'safety_monitor',
    'vanguard_battery_monitor', 'platform_diagnostics', 'scanner_safety_adapter',
    'scanner_evaluation_projection', 'robot_state_publisher', 'sick_safetyscanners2_node',
    'socket_can_sender_node_exe', 'socket_can_receiver_node_exe', 'paced_socket_can_sender',
}
for process in pathlib.Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        args = process.joinpath('cmdline').read_bytes().split(b'\0')
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if b'launch' in args and b'mobotic_bringup' in args and b'moboterra.launch.py' in args:
        raise SystemExit(f'Stop the existing MoboTerra launch (PID {process.name}) first.')
    if (b'--ros-args' in args and any(a == b'__ns:=/moboterra' or a.startswith(b'__ns:=/moboterra/') for a in args)
            and any(pathlib.Path(a.decode(errors='replace')).name in node_executables for a in args[:2])):
        try:
            environment = dict(item.split(b'=', 1) for item in process.joinpath('environ').read_bytes().split(b'\0') if b'=' in item)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if environment.get(b'ROS_DOMAIN_ID', b'0') == b'42':
            raise SystemExit(f'Stop the existing/orphaned MoboTerra node (PID {process.name}) before relaunching.')
with socket.socket() as listener_check:
    listener_check.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        listener_check.bind(('0.0.0.0', 9100))
    except OSError as error:
        raise SystemExit(f'FlexiSoft TCP port 9100 is unavailable: {error}. Stop the previous listener before relaunching.')
can_settings = Configuration().platform['can']
for interface, bitrate in dict((can_settings[role]['interface'], can_settings[role]['bitrate'])
                               for role in ('drives', 'battery')).items():
    info = json.loads(subprocess.check_output(['ip', '-json', '-details', 'link', 'show', interface]))[0]
    can = info['linkinfo']['info_data']
    if ('UP' not in info['flags'] or can.get('bittiming', {}).get('bitrate') != bitrate
            or 'LISTEN-ONLY' in can.get('ctrlmode', []) or can.get('state') == 'BUS-OFF'):
        raise SystemExit(f'{interface} must be UP at {bitrate} bit/s, transmitting and not BUS-OFF.')
PY
receiver_transmit_enabled=$(python3 - <<'PY'
from mobotic_config.configuration import Configuration
cfg = Configuration()
print('true' if cfg.parameters('manual')['receiver_transmit_enabled'] else 'false')
PY
)
# One-time PAIL recovery, after all stopped-stack/CAN checks and before ROS
# CAN writers start. A failed clear/verification aborts launch via set -e.
if [[ "$receiver_transmit_enabled" == true ]]; then
  echo 'Checking/clearing PAIL receiver error; keep sticks neutral and B2 released.'
  python3 "$PROJECT_ROOT/tools/pi/clear_joystick_error.py" --stack-lock-fd 9
else
  echo 'PAIL receive-only: skipping receiver startup reads, reset and NMT start.'
fi
fastdds shm clean
exec python3 -m mobotic_bringup.shutdown_guard --platform-config "$profile" --interface "$drive_interface" -- \
  ros2 launch mobotic_bringup moboterra.launch.py \
  stack_type:=hardware \
  kinematics_backend:=romb \
  start_description:=true start_odometry:=true \
  start_rviz:=true rviz_config:="$config_dir/live.rviz" \
  "$@" start_scanners:=true
