# mobotic_vanguard_battery

Read-only Vanguard J1939 telemetry monitoring. The node's `can_rx` subscription
is remapped by both launch files to `battery_can/can_rx`, isolated from drive CAN.
Publishes:

- `battery/state` (`sensor_msgs/msg/BatteryState`): one canonical public battery.
- `battery/system_state` (`mobotic_interfaces/msg/BatterySystemState`): readiness
  and minimum SOC across the configured internal participants; consumed by the supervisor.

There are no `battery/<id>/state` topics. One battery system may still contain
several internal CAN participants. The supplied reference configuration monitors
the controller plus four participants; verify their PGNs/source addresses against
the installed system. If fewer are installed, reduce all `batteries.*` arrays together.

Stale/missing/invalid SOC or electrical fields become NaN and block aggregate
readiness. Stale status marks the participant absent and reports watchdog expiry.
Reserved J1939 values must not appear as valid 100% SOC.
When `can_rx.header.stamp` is nonzero, each status/SOC/electrical sample retains
its CAN acquisition time. Delayed delivery consumes `telemetry_timeout` (default
0.5 s) rather than starting another full timeout at callback time. Already stale
frames and timestamps more than 100 ms ahead are rejected. Smaller future skew
is clamped to reception time, without granting extra freshness. Duplicate
timestamped frames and out-of-order samples cannot renew or overwrite a newer
sample; genuinely newer telegrams can restore readiness.

Zero-stamped CAN frames retain reception-time fallback for transport compatibility.
Replaying unstamped old payloads cannot be distinguished from new acquisitions;
use a bridge that preserves acquisition timestamps for delayed/replay detection.
Output headers remain report times; readiness and NaN/stale fields reflect the
underlying acquisition age, not just output publication time.
`percentage` and `minimum_percentage` are fractions; the supervisor enables only
when all required participants are fresh/ready and minimum SOC is at least `0.2` (20%).

The supervisor does not subscribe to public `battery/state`; its gate uses the
aggregate `battery/system_state`. External monitoring can consume either.

Contactor and charge control are intentionally excluded. ROS telemetry does not
authorize inferred battery-control commands.

Both integrated bringup and the standalone launch start a receive-only SocketCAN
receiver on `can1` by default; no battery CAN sender is created. Configure the
OS battery adapter for 500 kbit/s and listen-only mode. The receiver preserves
CAN acquisition timestamps (`use_bus_time=true`).

```bash
ros2 launch mobotic_vanguard_battery vanguard_battery.launch.py \
  robot_name:=moboterra can_interface:=can1
```

Use `start_socketcan:=false` for an already running external battery receiver.
It must publish on `/moboterra/battery_can/can_rx`, not the drive's `can_rx`.
In integrated bringup, the corresponding switch is `start_battery_socketcan`.
Do not run both launch files simultaneously for the same battery/namespace.
See [mapping and commissioning](../../docs/hardware_monitoring.md).
