# About

A ROS2 package that contains the driver for **Vanguard Batteries** from Briggs-Stratton using the CAN bus.

This package implements the communication with Vanguard batteries using the CAN bus. The battery uses `J1939` protocol
for CAN communication. The driver is implemented as a ROS2
node and uses the [SocketCAN](https://en.wikipedia.org/wiki/SocketCAN) interface to communicate with the CAN bus. The
driver provides the following functionalities:

- Reading battery status information via CAN PGNs.
- Publishing battery status information on ROS 2 topics.
- Sending commands to the battery.

# Driver Parameters

The driver is configured using launch arguments, which can be overridden at launch time.

| Launch Argument               | Description                                                            | Default Value                               | Node Parameter Name           |
|:------------------------------|:-----------------------------------------------------------------------|:--------------------------------------------|:------------------------------|
| `robot_name`                  | Namespace for the node.                                                | `(Empty String)` (via `ROBOT_NAME` env var) | `(Namespace)`                 |
| `delay`                       | Delay before starting the node (in seconds).                           | `0.0`                                       | `(Launch Prefix)`             |
| `can_device`                  | CAN device name (e.g., `can0`).                                        | `can0`                                      | `can_device`                  |
| `source_address`              | The CAN source address for the driver node (Hex string).               | `0xD0`                                      | `source_address`              |
| `high_voltage_publish_period` | High voltage publish period (in seconds).                              | `0.02`                                      | `high_voltage_publish_period` |
| `batteries.ids`               | List of string identifiers for the batteries.                          | `["parallel_controller"]`                   | `batteries.ids`               |
| `batteries.status_pgns`       | List of PGNs (Hex strings) to request for battery status.              | `["0xF096"]`                                | `batteries.status_pgns`       |
| `batteries.source_addresses`  | List of CAN source addresses for the batteries (Hex strings).          | `["0xF3"]`                                  | `batteries.source_addresses`  |
| `op_consent_duration`         | Duration of sending 'operational consent' to the battery (in seconds). | `1.5`                                       | `op_consent_duration`         |

**Note:** The lists for `batteries.ids`, `batteries.status_pgns`, and `batteries.source_addresses` are expected to
correspond to each other (e.g., if there are multiple batteries).

# Run the driver

1. Launch the driver. You can set the namespace using the `ROBOT_NAME` environment variable or by overriding the
   `robot_name` launch argument.

```bash
ROBOT_NAME=<your_robot_name> ros2 launch vanguard_battery_driver vanguard_driver.launch.py
```

# Published Topics

- `battery_status` - The battery status (`sensor_msgs/BatteryState`).
- `can_tx` - The CAN messages that are sent to the CAN bus (`can_msgs/frame`).

# Subscribed Topics

- `can_rx` - The CAN messages that are received from the batteries (`can_msgs/frame`).

# Specifics

1. The driver publishes commands at `high_voltage_publish_period`. The commands must be published at the specified rate,
   if the publishing stops, the battery firmware will initiate a shutdown.
2. For the high voltage to be initiated the commands must be following (1. op_consent: 3, high_voltage: 3 -> 2.
   op_consent: 1, high_voltage: 0 3. -> op_consent: 1, high_voltage: 1). Groups 1. and 2. are published for
   `op_consent_duration` each. Group 3. is published for the remainder of the node's lifecycle and only if the battery
   is in `operational` state after publishing group 2 (aligned with battery's specifications).
3. The `battery_state` currently ONLY contains info if the high-voltage has been initialized (`battery_state.present` =
   `True`).

# Experimental charger gateway

## Read-only battery-bus monitor

The `vanguard_charge_monitor` launch starts a SocketCAN receiver on `can1`
under `battery_can/can_rx`. It deliberately creates no SocketCAN sender or
`can_tx` publisher. It decodes:

- BMS charge command `0x18FF10F3`: mode and requested current.
- Charger status `0x0CFFBC49`: output voltage/current, charger state, AC
  connection, AC voltage, charging-active and fault flags.
- Charger limits `0x0CFFBD49`: voltage limit, current limit and power rating.
- Parallel battery controller SOC (`PGN 0xF091`, source `0xF3`).
- Parallel battery controller pack voltage/current (`PGN 0xF090`, source `0xF3`).

Each message group also has a `fresh`/`status_fresh`/`limits_fresh` topic that
becomes false after `stale_timeout_sec` (default 0.75 s). Run it separately
from the vehicle stack:

```bash
ROBOT_NAME=mobotic ros2 launch vanguard_battery_driver \
  vanguard_charge_monitor.launch.py can_interface:=can1
```

The monitor cannot enable or inhibit charging and therefore does not create a
duplicate J1939 source address on the battery bus.

The monitor can also write timestamped JSON arrays compatible with the style of
`can_rosbag_recorder`. Recording is disabled by default. Start and stop it with:

```bash
ros2 service call /mobotic/vanguard_charge_monitor/start_recording std_srvs/srv/Trigger
ros2 service call /mobotic/vanguard_charge_monitor/stop_recording std_srvs/srv/Trigger
```

Files default to
`can_telemetry_records/vanguard_charge_YYYYMMDD_HHMMSS.json`. Each snapshot
contains BMS mode/requested current; battery SOC, pack voltage/current; charger
output voltage/current, status, AC state and fault; charger limits; and
freshness flags. The battery values are also published as `battery/soc_percent`,
`battery/pack_voltage`, and `battery/pack_current`; current sign follows the BMS
CAN convention. To record immediately:

```bash
ros2 launch vanguard_battery_driver vanguard_charge_monitor.launch.py \
  record_on_start:=true recording_dir:=can_telemetry_records recording_hz:=10.0
```

For normal machine telemetry, do not enable this separate recorder. The
platform `debug/can_rosbag_recorder.py` decodes the same Vanguard CAN frames
from `/mobotic/battery_can/can_rx` and stores battery, BMS charge-command,
charger, and EFF3 controller fields in each `can_telemetry_*.json` record.


## Combined platform and battery telemetry recording

Use the combined launch entry instead of starting the Vanguard monitor separately:

```bash
ROBOT_NAME=mobotic ros2 launch mobotic_launch can_rosbag_recorder.launch.py
```

This starts the read-only `can1` receiver and Vanguard monitor, then records
platform and battery telemetry together in one
`can_telemetry_records/can_telemetry_YYYYMMDD_HHMMSS.json` file. The Vanguard
monitor's own recorder is disabled by this launch file.

Stop the launch with Ctrl-C to close the JSON array cleanly. To start the nodes
without immediately opening a file, pass `recorder_record_on_start:=false`.

## SOC-only EFF3 charge-rate controller

`vanguard_charge_rate_controller` implements the Vanguard Custom Charge Rate
Request from "Third Party Charging and Custom Charge Rate Request", Rev B. It
broadcasts EFF3 every 250 ms on the existing single CAN bus. It does not replace
or impersonate the BMS FF10 charger command; the BMS remains responsible for
the permitted charge current and charger control.

The controller uses only battery SOC from `can_rx`. Its hysteresis gate enables
charging when SOC reaches 50% or below, keeps charging enabled through the band,
and disables it at 80% or above. Stale or malformed SOC commands 0 A.

Requested current is the minimum of `maximum_c_rate * system_capacity_ah`,
`absolute_current_limit_a`, and the EFF3 protocol limit. The maximum configured
C-rate is restricted to 0.5.

For the one-pack configuration, `system_capacity_ah=67.5` and 0.5C limit the
current to 33.75 A. This logic has no dependency on platform motion, navigation
routes, or stop-before-turn behavior.

`system_capacity_ah` must be the total capacity of the complete parallel battery
system. The policy library keeps a zero-capacity safe default; the launch file
configures one 67.5 Ah Fi3.5 pack but retains the disabled-by-default commissioning
interlock. After bench validation, start the one-pack controller with:

```bash
ros2 launch vanguard_battery_driver vanguard_charge_rate_controller.launch.py \
  robot_name:=mobotic enable_controller:=true system_capacity_ah:=67.5
```

State is published at 4 Hz with reliable, transient-local QoS on
`charge_rate/requested_current`, `charge_rate/charging_requested`, and
`charge_rate/status`. This feature requires Universal Turnkey firmware r2.2 or later.
The document does not specify the BMS behavior if EFF3 transmission stops, so
this node is an operational control, not an independent safety interlock.
