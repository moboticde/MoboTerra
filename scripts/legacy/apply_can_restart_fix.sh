#!/usr/bin/env bash
set -euo pipefail
[[ "$EUID" -eq 0 ]] || { echo 'Run this script with sudo.' >&2; exit 1; }
source_file=/home/mobotic/Downloads/MoboTerra-main/deploy/host/moboterra_supervisor.py
target_file="$(readlink -f /opt/moboterra/current/moboterra_supervisor.py)"
case "$target_file" in /opt/moboterra/releases/*/moboterra_supervisor.py) ;; *) echo 'Unexpected installed path; refusing update.' >&2; exit 1 ;; esac
[[ "$(sha256sum "$source_file" | cut -d ' ' -f 1)" == c2ff21a7815a771d114bdd1151126a6a63f25a928b7d4b3000d5f0a53fe97b7d ]] || { echo 'Tested source changed; refusing update.' >&2; exit 1; }
installed_sha="$(sha256sum "$target_file" | cut -d ' ' -f 1)"
if [[ "$installed_sha" != c2ff21a7815a771d114bdd1151126a6a63f25a928b7d4b3000d5f0a53fe97b7d ]]; then
    [[ "$installed_sha" == e9721f0a349983c937f1d16bb3c959a97fbbd7a88b005dc250656081733ec4e6 ]] || { echo 'Installed supervisor changed; refusing update.' >&2; exit 1; }
    python3 -c 'from pathlib import Path; import sys; compile(Path(sys.argv[1]).read_text(), sys.argv[1], "exec")' "$source_file"
    backup_file="${target_file}.before-can-restart-fix-$(date -u +%Y%m%dT%H%M%SZ)"
    cp --preserve=all -- "$target_file" "$backup_file"
    staging_file="$(mktemp "${target_file}.XXXXXX")"
    install -m 0755 -- "$source_file" "$staging_file"
    mv -f -- "$staging_file" "$target_file"
    echo "Applied tested CAN fix. Backup: $backup_file"
else
    echo 'Tested CAN fix is already installed.'
fi
systemctl restart moboterra-supervisor.service
systemctl is-active moboterra-supervisor.service
ip -details link show can0
ip -details link show can1
echo 'Supervisor restarted. Check its health after startup with: sudo moboterra-ctl status'
