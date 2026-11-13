#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source "$PROJECT_ROOT/scripts/setup_ros.bash"
exec python3 "$PROJECT_ROOT/tools/pi/receiver_diagnostic.py" "$@"
