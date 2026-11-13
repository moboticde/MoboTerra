#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)"
exec "$PROJECT_ROOT/scripts/start_ros2_four_micontrol.sh" "$@"
