#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /opt/moboterra/setup.bash
set -u

pgrep -f '/opt/moboterra/lib/mobotic_supervisor/mobotic_supervisor' >/dev/null
pgrep -f '/opt/moboterra/lib/mobotic_bringup/platform_diagnostics' >/dev/null

robot_namespace="${ROBOT_NAME-moboterra}"
robot_prefix=""
[[ -z "${robot_namespace}" ]] || robot_prefix="/${robot_namespace}"
required_csv="${MOBOTERRA_REQUIRED_NODES:-${robot_prefix}/mobotic_supervisor,${robot_prefix}/platform_diagnostics}"
nodes="$(timeout 8 ros2 node list --no-daemon --spin-time 3 2>/dev/null)"

IFS=',' read -r -a required_nodes <<<"${required_csv}"
for required in "${required_nodes[@]}"; do
    if ! grep -Fxq "${required}" <<<"${nodes}"; then
        echo "Required ROS node is missing: ${required}" >&2
        exit 1
    fi
done

