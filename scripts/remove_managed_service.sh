#!/usr/bin/env bash
# Remove only MoboTerra's systemd registration; preserve Docker and project files.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
service=moboterra-supervisor.service
unit_path=/etc/systemd/system/moboterra-supervisor.service
expected="$PROJECT_ROOT/config/systemd/moboterra-supervisor.service"

if [[ -L "$unit_path" ]]; then
    if [[ "$(readlink -f -- "$unit_path")" != "$expected" ]]; then
        echo "Unexpected service link; refusing to remove: $unit_path" >&2
        exit 1
    fi
elif [[ -e "$unit_path" ]]; then
    echo "Expected a project-owned service link; refusing to delete a different unit file." >&2
    exit 1
fi

if [[ "${1:-}" == "--check" ]]; then
    echo "Service registration: $unit_path"
    systemctl is-enabled "$service" || true
    systemctl is-active "$service" || true
    echo 'Check only: no services or files changed.'
    exit 0
fi
if [[ "$EUID" -ne 0 ]]; then
    echo "Run: sudo $PROJECT_ROOT/scripts/remove_managed_service.sh" >&2
    exit 1
fi

# Stop while the linked unit is still available. Disabling a linked unit
# can remove the unit link itself as well as the boot-target links.
load_state="$(systemctl show "$service" --property=LoadState --value 2>/dev/null || true)"
if [[ "$load_state" != not-found ]]; then
    systemctl stop "$service"
fi
if [[ -L "$unit_path" ]]; then
    systemctl disable "$service"
    rm -f -- "$unit_path"
fi
systemctl daemon-reload
systemctl reset-failed "$service" 2>/dev/null || true
load_state="$(systemctl show "$service" --property=LoadState --value 2>/dev/null || true)"
if [[ "$load_state" != not-found ]]; then
    echo "Service still resolves as '$load_state'; inspect systemctl cat $service." >&2
    exit 1
fi
mkdir -p "$PROJECT_ROOT/runtime"
python3 - "$PROJECT_ROOT" <<'PY'
import json, sys
from pathlib import Path
root = Path(sys.argv[1])
path = root / 'runtime/system-settings-installation.json'
state = json.loads(path.read_text()) if path.exists() else {}
state['managed_systemd_service'] = False
state['managed_service_removed'] = True
state.get('links', {}).pop('/etc/systemd/system/moboterra-supervisor.service', None)
path.write_text(json.dumps(state, indent=2) + '\n')
PY
echo 'MoboTerra removed from systemd. Docker and project configuration were retained.'
