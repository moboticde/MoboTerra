#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
source "$PROJECT_ROOT/scripts/setup_ros.bash"
if [[ "${1:-}" == "--check" ]]; then
    python3 - <<'PY'
from rosidl_runtime_py.utilities import get_message
for name in ('OperatorState', 'VehicleModeState', 'WheelModuleStatusArray', 'WheelModuleCommandArray'):
    get_message('mobotic_interfaces/msg/' + name)
print('Recording environment and custom message types are available; recording not started.')
PY
    test -r "$PROJECT_ROOT/config/raspberry-pi/bag-qos.yaml"
    exit 0
fi
mkdir -p "$PROJECT_ROOT/runtime/bags"
exec ros2 bag record -s mcap \
    -o "$PROJECT_ROOT/runtime/bags/steering_$(date +%Y%m%d_%H%M%S)" \
    --qos-profile-overrides-path "$PROJECT_ROOT/config/raspberry-pi/bag-qos.yaml" \
    --topics \
    /moboterra/tf /moboterra/tf_static /moboterra/joint_states \
    /moboterra/kinematics/joint_setpoints \
    /moboterra/wheel_modules/command /moboterra/wheel_modules/status \
    /moboterra/operator/cmd_vel /moboterra/operator/state /moboterra/joy \
    /moboterra/vehicle/cmd_vel /moboterra/vehicle/mode \
    /moboterra/can_rx /moboterra/can_tx
