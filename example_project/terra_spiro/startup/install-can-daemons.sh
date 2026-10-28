#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

if (( EUID != 0 )); then
  echo "Run this installer as root (for example: sudo $0)." >&2
  exit 1
fi

systemctl disable --now robot-startup.service robot-can0.service robot-can1.service robot-can.service 2>/dev/null || true
rm -f /etc/systemd/network/10-peak-can0.link
rm -f /etc/systemd/network/11-canable-can1.link
rm -f /etc/systemd/system/robot-can0.service
rm -f /etc/systemd/system/robot-can1.service

install -m 0644 "$script_dir/99-robot-can.rules" /etc/udev/rules.d/99-robot-can.rules
install -m 0755 "$script_dir/configure-robot-can.sh" /usr/local/sbin/configure-robot-can
install -m 0644 "$script_dir/robot-can.service" /etc/systemd/system/robot-can.service

systemctl daemon-reload
udevadm control --reload-rules
systemctl reset-failed
systemctl enable --now robot-can.service

echo "Installed persistent CAN naming and daemon configuration."
echo "Verify with: systemctl status robot-can.service"
