# Supervisor interface contract

The hardware-facing and operator-facing packages publish stable inputs for `mobotic_supervisor`. The supervisor selects
the active command source and publishes either the final supervised `cmd_vel` consumed by `mobotic_kinematics` or the
validated `supervisor/joint_setpoints` consumed by `mobotic_driver`.

All names are relative to the robot namespace (default `/moboterra`).
This is an implementation contract, not a claim of completed hardware commissioning.

Configured powered modules are `front_left`, `front_right`, `rear_left` and
`rear_right`, with no casters. Each has `<module>_steering` and `<module>_traction`
joints. Direct autonomy commands and joint feedback must contain all eight
configured joint names; wheel status must contain all four module names. An
old two-module `front_*`/`rear_*` command is not a valid MoboTerra command.
Standstill verification checks velocities of all eight joints, not just body motion.

Cached command and readiness inputs must remain fresh by **both** local receipt
time and original `header.stamp` whenever used, not just when received. Delivery
latency consumes the configured timeout rather than restarting it. Zero stamps
and negative receipt ages are invalid; the existing 100 ms future-header skew
allowance is retained, except for strict non-future standstill proof.

| Producer | Topic | Type | Supervisor use |
|:--|:--|:--|:--|
| `mobotic_manual_control` | `manual/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Manual velocity candidate |
| `mobotic_manual_control` | `manual/state` | `mobotic_interfaces/msg/ManualControlState` | Fresh connected/active source; optional deadman telemetry |
| External autonomy | `autonomy/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Autonomous velocity candidate |
| External autonomy | `autonomy/joint_setpoints` | `sensor_msgs/msg/JointState` | Optional direct joint-level candidate |
| `mobotic_safety` | `safety/state` | `mobotic_interfaces/msg/SafetyState` | Authoritative ROS safety-monitoring input |
| `mobotic_safety` | `safety/io_state` | `mobotic_interfaces/msg/SafetyIOState` | Decoded interlocks, known/unknown signal flags and raw FlexiSoft byte |
| `mobotic_vanguard_battery` | `battery/system_state` | `mobotic_interfaces/msg/BatterySystemState` | Whole-pack freshness and readiness |
| `mobotic_driver` | `wheel_modules/status` | `mobotic_interfaces/msg/WheelModuleStatusArray` | Drive readiness and faults |
| `mobotic_kinematics` | `agv_vel` | `geometry_msgs/msg/TwistStamped` | Fresh measured velocity for standstill verification |
| `mobotic_driver` | `joint_states` | `sensor_msgs/msg/JointState` | Complete feedback, additional joint-speed standstill check |
| Joystick/operator | `vehicle/mode_request` | `mobotic_interfaces/msg/VehicleModeRequest` | Timestamped MANUAL/AUTO_VELOCITY request only |

The supervisor output contract is:

| Topic | Type | Consumer |
|:--|:--|:--|
| `cmd_vel` | `geometry_msgs/msg/TwistStamped` | `mobotic_kinematics` |
| `vehicle/mode_state` | `mobotic_interfaces/msg/VehicleModeState` | Manual-control toggle feedback, driver source authority, operator UI and system nodes |
| `supervisor/joint_setpoints` | `sensor_msgs/msg/JointState` | `mobotic_driver` in optional `AUTO_DIRECT` mode |
| `wheel_modules/command` | `mobotic_interfaces/msg/WheelModuleCommand` | `mobotic_driver`, global enable/disable and error clearing |

`battery/state` is the single public `sensor_msgs/msg/BatteryState` telemetry output;
the supervisor does **not** subscribe to it. It uses `battery/system_state` for
freshness/readiness/minimum SOC across configured internal participants.
The battery monitor accounts for CAN acquisition timestamps before producing
that readiness: delayed/replayed timestamped telegrams cannot reset its freshness
budget. Aggregate headers are report times, not battery sample times. Zero-stamped
CAN transports fall back to reception time and cannot detect unstamped replay.

The `vehicle/set_mode` (`mobotic_interfaces/srv/SetVehicleMode`) service is the request/response alternative to
`vehicle/mode_request` for modes 0/1; only the service accepts AUTO_DIRECT (2).
The temporary F310/F710 Back button toggles 0/1 using fresh supervisor state;
from mode 2 it requests mode 0. Face buttons are unused. This is API separation,
not authentication of terminal clients. A request blocks both candidate sources and stops through
the outgoing source's command path. Velocity commands ramp to zero; outgoing
direct joint commands stop immediately. The switch requires ready hardware and
fresh `agv_vel` plus `joint_states` continuously below configured thresholds for
0.25 s. Moving, missing, invalid, or stale feedback resets this confirmation.
A 10 s timeout keeps the old mode active and motion blocked until an explicit
mode request retries the transition. `vehicle/mode_state` remains authoritative.
Joystick mode requests and terminal service requests use this same state machine.
Reselecting an active mode also stops/verifies; pending conflicting requests are
rejected. AUTO_DIRECT selection has no extra configuration permission switch.
The driver also consumes `vehicle/mode_state` and permits only the selected
kinematics (0/1) or direct (2) path. Missing/stale mode authority stops motion,
and delayed pre-switch commands cannot take over the new mode.

## Autonomy with its own kinematics

Velocity candidates on `autonomy/cmd_vel` (and `manual/cmd_vel`) require
`TwistStamped.header.frame_id` exactly matching the supervisor's `output_frame_id`,
default `base_link`. Empty frames, `map`, `odom`, and other mismatches are rejected;
that source's cached velocity is cleared and motion stops on the next control
cycle if it is selected. Valid input is required to resume. The supervisor never
transforms or silently relabels an external velocity frame. An autonomy stack
using another frame must convert its velocity into the configured body frame
before publishing. Keep manual control and kinematics frame settings consistent
with the supervisor. Direct joint setpoints are validated by joint names, not
this velocity-frame rule.

An external autonomy stack may subscribe directly to the driver's read-only `joint_states` feedback. Its own kinematics
may publish `sensor_msgs/msg/JointState` commands on `autonomy/joint_setpoints` in `AUTO_DIRECT` mode. The supervisor
validates the timestamp, exact joint set, finite values, safety state, configured traction mode, and limits before
republishing the command on `supervisor/joint_setpoints`.

Steering angles are rejected unless their rounded encoder positions fit signed
32-bit CAN ticks. `direct_steering_encoder_resolutions` follows
`expected_steering_joint_names` order and must match the steering entries in the
driver's `drives.resolutions`. Invalid candidates clear the cached autonomy joint
command; the selected direct path stops on the next supervisor cycle. Stop/hold
feedback is checked against the same range. The driver preflights the whole batch
on both motion inputs using its own resolutions and checks again before integer
conversion. Invalid driver commands do not renew its motion watchdog. Valid
multi-turn positions are not wrapped; mechanical travel limits remain unverified.

The driver accepts joint motion only on `kinematics/joint_setpoints` and
`supervisor/joint_setpoints`. It has no generic public `joint_setpoints`
subscription. External autonomy must use the supervised input, not publish on
internal driver inputs. Topic naming/routing alone is not DDS access control.

`wheel_modules/command` is a separate global management interface for enabling/disabling all modules and clearing all
drive errors. Enabling also releases all brakes; disabling engages them. It never carries motion setpoints.
`wheel_modules/status` remains the aggregated module feedback interface.
The supervisor publishes the desired enable state every control cycle: true only
while safety and battery are fresh and ready with SOC at least 20%. Missing or
stale state, safety inhibition, battery faults, disconnected high voltage, or SOC
below 20% produce false. Wheel status and joystick deadman do not determine
enable; they additionally determine whether motion commands may be forwarded.
Normal supervisor commands leave `clear_errors=false`.
The driver still clears errors during initialization/enable sequences; a reviewed
explicit fault-reset/retry policy remains outstanding. Hardware brake behavior
must be verified independently: `brake_state_known=false`.

## Fail-safe expectations

- Missing or stale `safety/state` prohibits motion.
- Missing or stale `safety/io_state` prohibits motion; decoded E-stop, safety-enable, STO and field interlocks are checked. Unknown independent STO/enable mappings are explicitly flagged, not inferred as confirmation.
- `SafetyState.motion_permitted == false` prohibits motion.
- Missing, stale, absent, unhealthy or invalid required aggregate battery telemetry prohibits drive enable; minimum SOC below 0.2 also inhibits enable.
- Freshly published wheel status with `feedback_fresh=false` prohibits motion:
  the driver's cached enabled state cannot hide loss of required CAN feedback.
- Manual mode requires fresh `manual/state` with `connected` and `command_active`
  true, plus a fresh manual velocity command. The USB profile has no held-button
  requirement; optional teleop deadman gating is included in `command_active`,
  while `deadman_pressed` remains physical-button telemetry.
- Joystick timeout clears the manual node's stored command and acceleration
  timing; neutral input after reconnect cannot revive the previous velocity.
- Autonomous velocity mode requires a fresh autonomous command; autonomous planning remains outside this repository.
- Autonomous direct mode also requires fresh, complete `joint_states` feedback from the driver.
- Direct-joint autonomous mode is selectable as mode 2 without extra opt-in, but motion is inhibited during reduced-speed safety states.
- Mode transitions verify sustained vehicle and joint standstill before changing sources; feedback loss or timeout keeps motion blocked.
- After a mode switch, only newly timestamped commands may resume motion.
- Safety, battery, or wheel-readiness loss clears all supervisor motion buffers;
  direct-mode joint-feedback loss or reduced-speed inhibition does the same.
  Commands received while inhibited are discarded. After recovery, a source
  command timestamped at or after recovery is required on the selected input.
- Driver joint-command loss (default 0.1 s) requests zero motion. Independent supervisor-management loss (0.3 s) requests stop + disable even if joint commands continue.
- Driver decoded motion/status loss (0.3 s) or required fault-data loss (2.5 s) blocks all-unit motion independently of the supervisor. Recovery requires a newly timestamped command.

`wheel_modules/status` headers are report times; `feedback_fresh` describes
underlying CAN validity. `joint_states` uses the oldest required motion sample
time, and `agv_vel` preserves that timestamp for standstill checks.

Diagnostics are shared on `diagnostics`; see [hardware monitoring](hardware_monitoring.md).

ROS safety messages are monitoring data. The hardware safety controller and STO wiring remain the safety-rated stop path.
