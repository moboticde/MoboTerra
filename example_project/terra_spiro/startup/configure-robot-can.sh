#!/usr/bin/env bash
set -euo pipefail

# Stable adapter identities reported by udev. These may be overridden from the
# service environment if an adapter is replaced.
platform_can_serial=${PLATFORM_CAN_SERIAL:-0040001D5847570D20343432}
battery_can_serial=${BATTERY_CAN_SERIAL:-001B00305346571320373231}
can_bitrate=${CAN_BITRATE:-500000}

interface_for_serial() {
  local expected_serial=$1
  local net_path
  local serial

  for net_path in /sys/class/net/*; do
    [[ -e $net_path/device ]] || continue
    serial=$(udevadm info --query=property --path="$net_path" 2>/dev/null |
      sed -n 's/^ID_SERIAL_SHORT=//p')
    if [[ $serial == "$expected_serial" ]]; then
      basename "$net_path"
      return 0
    fi
  done
  return 1
}

platform_interface=
battery_interface=
for ((attempt = 0; attempt < 50; ++attempt)); do
  platform_interface=$(interface_for_serial "$platform_can_serial" || true)
  battery_interface=$(interface_for_serial "$battery_can_serial" || true)
  if [[ -n $platform_interface && -n $battery_interface ]]; then
    break
  fi
  sleep 0.2
done

if [[ -z $platform_interface || -z $battery_interface ]]; then
  echo "Required CANable adapters were not both found." >&2
  echo "Platform serial: $platform_can_serial; battery serial: $battery_can_serial" >&2
  exit 1
fi

if [[ $platform_interface == "$battery_interface" ]]; then
  echo "Platform and battery serials resolved to the same interface." >&2
  exit 1
fi

ip link set dev "$platform_interface" down
ip link set dev "$battery_interface" down

# Move both devices out of the can0/can1 namespace before assigning roles.
if [[ $platform_interface != can_plat_tmp ]]; then
  ip link set dev "$platform_interface" name can_plat_tmp
fi
if [[ $battery_interface != can_batt_tmp ]]; then
  ip link set dev "$battery_interface" name can_batt_tmp
fi

ip link set dev can_plat_tmp name can0
ip link set dev can_batt_tmp name can1

ip link set dev can0 type can bitrate "$can_bitrate"
ip link set dev can0 txqueuelen 1000
ip link set dev can0 up

ip link set dev can1 type can bitrate "$can_bitrate" listen-only on
ip link set dev can1 txqueuelen 1000
ip link set dev can1 up

echo "Configured can0=CANable platform ($platform_can_serial) and can1=CANable battery ($battery_can_serial, listen-only) at $can_bitrate bit/s."
