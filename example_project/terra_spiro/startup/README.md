# CAN startup daemons

This setup gives the two identical CANable adapters stable roles by USB serial,
independent of USB enumeration order:

| Interface | CANable serial | Role | Configuration |
| --- | --- | --- | --- |
| `can0` | `0040001D5847570D20343432` | Mobile-platform control | 500 kbit/s |
| `can1` | `001B00305346571320373231` | Battery telemetry | 500 kbit/s, listen-only |

Install the identity-aware naming script and systemd service with:

```bash
sudo ./startup/install-can-daemons.sh
```

The installer disables the old startup and CAN services and removes obsolete
direct-name `.link` rules. The unified service safely moves both adapters to
temporary names before assigning their serial-based roles.

After installation, verify the mapping and bus state:

```bash
udevadm info --query=property --path=/sys/class/net/can0 | grep -E 'ID_NET_DRIVER|ID_SERIAL_SHORT'
udevadm info --query=property --path=/sys/class/net/can1 | grep -E 'ID_NET_DRIVER|ID_SERIAL_SHORT'
ip -details -statistics link show can0
ip -details -statistics link show can1
systemctl status robot-can.service
```

Both interfaces should use `gs_usb`; `can0` should be active normally and
`can1` should show `listen-only on`. Start the battery monitor separately:

```bash
ROBOT_NAME=mobotic ros2 launch vanguard_battery_driver \
  vanguard_charge_monitor.launch.py can_interface:=can1
```

The receiver publishes frames on `/mobotic/battery_can/can_rx`. The telemetry
recorder subscribes to that topic in addition to `/mobotic/can_rx` for drive data.

If an adapter is replaced, set its new serial in `/etc/default/robot-can`:

```bash
PLATFORM_CAN_SERIAL=0040001D5847570D20343432
BATTERY_CAN_SERIAL=001B00305346571320373231
CAN_BITRATE=500000
```
