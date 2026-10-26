# mobotic_vanguard_battery

Read-only Vanguard J1939 telemetry monitoring. The node's `can_rx` subscription
is remapped by both launch files to `battery_can/can_rx`, isolated from drive CAN.
Publishes:

- `battery/battery1/state` and `battery/battery2/state`
  (`sensor_msgs/msg/BatteryState`): separate measurements for the two packs.
- `battery/state` (`sensor_msgs/msg/BatteryState`): compatibility alias for
  `primary_battery_id` (now `battery1`); it is not an aggregate of both packs.
- `battery/system_state` (`mobotic_interfaces/msg/BatterySystemState`): readiness
  and minimum SOC across the configured internal participants; consumed by the supervisor.

Both supplied platform profiles configure two fixed packs directly on one
500-kbit/s battery CAN bus, with no separate Parallel Battery Controller. Each
configured participant gets a `battery/<id>/state` publisher; IDs must be valid
ROS topic tokens. The monitor separates traffic by source address and PGN and
rejects duplicate telegram bindings so a single binding cannot satisfy two packs.

**The two packs' telemetry mapping still requires commissioning.** The supplied
`battery1` (`0xF4`) and `battery2` (`0xF5`) source addresses and their PGNs are
retained reference values from the previous controller-based setup. They are not
confirmed addresses for the installed fixed packs. Edit `battery.participants`
in the selected platform YAML using the vendor telemetry specification and a
passive CAN capture. `batteries.*` node parameters are derived from those records.
For an existing deployment, update the battery section of
`/etc/moboterra/platform.yaml` as well: installation preserves that commissioned
file, so rebuilding the image alone does not replace its old participant list.
The quick-start guide does not define status/SOC/electrical telegram layouts or
address assignment; do not infer them from the number of packs or the wiring.

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

Both packs must provide fresh status, SOC and electrical telegrams, with valid
values, operational status and HV connected. Either pack's loss/fault/HV loss
blocks readiness; either SOC below 20% blocks supervisor enable/motion. The
aggregate reports the lower SOC, never an average that could hide a depleted
pack. Per-pack current/voltage remain separate; no system current is synthesized.

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

## Two-pack commissioning

The supplied *Vanguard Battery Fixed Pack Quick Start Guide*, rev 3eu (08/2026),
applies to Universal-Turn-Key firmware. Figure 3 (page 9) shows two packs on a
shared bus; page 11 specifies J1939 at 500 kbit/s. The battery connector's CAN0
is the primary OEM bus (pins 4/5); its CAN1 (pins 15/16) is not for OEM use.
These battery connector names are separate from Linux device names: MoboTerra's
Linux `can1` receives the battery CAN0 bus. Follow the vendor installation guide
for wiring, termination, firmware compatibility and electrical enable/interlocks.

On a commissioned, listen-only Linux adapter, record and inspect traffic:

```bash
candump -L can1 > battery-two-packs.log
ros2 topic echo /moboterra/battery/battery1/state
ros2 topic echo /moboterra/battery/battery2/state
ros2 topic echo /moboterra/battery/system_state
```

Identify each pack independently under the approved commissioning procedure,
then confirm both together: source identity, PGNs, byte layout, scaling, status
codes and update periods must match the installed firmware. Two transmitters
with indistinguishable CAN identities cannot be separated by this monitor;
resolve addressing using the vendor procedure. The monitor does not transmit
address-assignment, contactor or charging commands. Verify that losing either
pack or any required telegram blocks readiness, and that either pack below 20%
inhibits motion. Local software tests do not establish physical acceptance.
