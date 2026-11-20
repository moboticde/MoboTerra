#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
# Preserve the original launcher's collision, CAN and port checks.
if [[ "${1:-}" == "--check" ]]; then
  exec "$PROJECT_ROOT/scripts/start_MoboTerra" "$@"
fi
exec "$PROJECT_ROOT/scripts/start_MoboTerra" \
  start_safety:=false start_battery:=false \
  "$@" control_mode:=manual_only
