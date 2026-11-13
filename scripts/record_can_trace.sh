#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
source "$PROJECT_ROOT/scripts/setup_ros.bash"
exec python3 "$PROJECT_ROOT/tools/pi/record_can_trace.py" "$@"
