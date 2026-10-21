#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
# Only rclpy graph discovery is used below. Sourcing the complete workspace
# repeats the ROS underlay setup and loads unrelated package hooks on each probe.
set -u

pgrep -f '/opt/moboterra/lib/mobotic_supervisor/mobotic_supervisor' >/dev/null
pgrep -f '/opt/moboterra/lib/mobotic_bringup/platform_diagnostics' >/dev/null

robot_namespace="${ROBOT_NAME-moboterra}"
robot_prefix=""
[[ -z "${robot_namespace}" ]] || robot_prefix="/${robot_namespace}"
required_csv="${MOBOTERRA_REQUIRED_NODES:-${robot_prefix}/mobotic_supervisor,${robot_prefix}/platform_diagnostics}"
# Avoid loading the complete ROS CLI/plugin set on every health probe. On the
# Pi that startup overhead can consume the discovery deadline by itself.
timeout 8 python3 - "${required_csv}" <<'PY'
import os
import sys
import time
import rclpy

required = {name.strip() for name in sys.argv[1].split(',') if name.strip()}
rclpy.init()
node = rclpy.create_node('moboterra_healthcheck_' + str(os.getpid()))
missing = required
try:
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        discovered = {namespace.rstrip('/') + '/' + name
                      for name, namespace in node.get_node_names_and_namespaces()}
        missing = required - discovered
        if not missing:
            break
finally:
    node.destroy_node()
    rclpy.shutdown()
for name in sorted(missing):
    print('Required ROS node is missing: ' + name, file=sys.stderr)
sys.exit(1 if missing else 0)
PY

