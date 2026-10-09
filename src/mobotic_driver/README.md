# mobotic_driver

C++ ROS 2 CAN driver for MoboTerra's four powered steering/traction wheel modules.
Steering uses the MiControl protocol; traction uses ELMO CANopen/DS402.
Commissioning must verify node IDs, scaling, gearing, limits and hardware brakes.

Use the integrated [bringup](../mobotic_bringup/README.md), not the standalone
driver launch for normal operation. The latter now loads the same validated `platform_config` and driver policy,
but does not start the supervisor or CAN bridge. Build from the
[repository root](../../README.md), including `mobotic_interfaces`.

## Topics

Names are relative to the namespace (normally `/moboterra`).

| Direction | Topic | Type | Purpose |
|:--|:--|:--|:--|
| Input | `can_rx` | `can_msgs/msg/Frame` | Received CAN frames |
| Input | `kinematics/joint_setpoints` | `sensor_msgs/msg/JointState` | Kinematics motion commands |
| Input | `supervisor/joint_setpoints` | `sensor_msgs/msg/JointState` | Supervised external joint commands |
| Input | `wheel_modules/command` | `mobotic_interfaces/msg/WheelModuleCommand` | Global management, not motion |
| Input | `vehicle/mode_state` | `mobotic_interfaces/msg/VehicleModeState` | Exclusive motion-path authority |
| Output | `can_tx` | `can_msgs/msg/Frame` | Drive requests, commands and CANopen SYNC |
| Output | `joint_states` | `sensor_msgs/msg/JointState` | Complete fresh measured joint feedback |
| Output | `wheel_modules/status` | `mobotic_interfaces/msg/WheelModuleStatusArray` | Per-module readiness and last-known faults |
| Output | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Watchdogs, CAN freshness/faults and raw safety-register health |

Joint motion uses radians, rad/s and amperes and is converted to backend controller
units. Joint names must match the kinematics and supervisor configuration.

Only **one** motion input is accepted: `kinematics/joint_setpoints` in MANUAL or
AUTO_VELOCITY, `supervisor/joint_setpoints` in AUTO_DIRECT. While switching, the
old mode retains its stop/ramp path until the supervisor proves standstill. A
mode/transition boundary rejects older queued commands; completion (including
same-mode reselection) clears targets. Recovery after mode-state loss similarly
requires a newly timestamped command. Mode state must be ordered and fresh by
both original ROS timestamp and steady receipt age (`supervisor_timeout`, 0.3 s).
Missing/stale state requests all-unit stop, without independently changing the
safety/battery enable rule. Mode IDs alone do not establish permission to move.
Both management and mode-state streams are required for standalone driver use.
Diagnostics include `/mode_authority`. This routing gate is not a safety-rated
interlock and assumes trusted ROS publishers on the internal topics.

Both motion inputs validate every steering angle against that drive's encoder
resolution before sending any command in the batch. The rounded target must fit
signed 32-bit CAN ticks (`-2147483648` through `2147483647`); non-finite values,
conversion overflow and out-of-range targets reject the entire batch without
renewing the motion watchdog. The backend also checks before integer conversion.
Valid multi-turn angles are not wrapped. These are wire-format limits, not verified
mechanical steering limits; commissioning must establish the latter separately.

## Global management and services

`WheelModuleCommand` contains a timestamp, `enable` and `clear_errors`.
It applies to every configured drive together. Enable requests initialize/enable
the units as needed; hardware brake release follows enable. Disable requests
stop/disable the units. There is no separate `brake_release` command.
Actual brake feedback is unavailable (`brake_state_known=false`).

The supervisor renews this command every control cycle. Repeated enable requests
preserve active targets; accepted commands must be fresh with strictly increasing
timestamps. Replays do not renew the management lease.

All services use `std_srvs/srv/Trigger`. These are namespace-relative names,
not node-private `~/...` services:

| Default full name | Purpose |
|:--|:--|
| `/moboterra/initialize_all_drives` | Initialize every drive |
| `/moboterra/enable_all_drives` | Enable every drive; requires a fresh supervisor enable lease |
| `/moboterra/disable_all_drives` | Stop and disable every drive |
| `/moboterra/clear_errors` | Clear errors on every drive |

A manual disable is not a persistent supervisor override: a subsequent ready
supervisor cycle can request enable again. Normal supervisor messages set
`clear_errors=false`; the driver still clears errors in initialization/enable
sequences. An explicit reset/retry policy remains a commissioning task.

## Independent watchdogs and feedback validity

| Parameter | Hardware default | Effect |
|:--|:--|:--|
| `watchdog_timeout` | 0.1 s | Missing accepted joint commands request zero motion |
| `supervisor_timeout` | 0.3 s | Management loss requests stop + disable and cancels initialization/enable retries |
| `motion_feedback_timeout` | 0.3 s | Reject stale velocity/steering position |
| `status_feedback_timeout` | 0.3 s | Reject stale drive status |
| `fault_feedback_timeout` | 2.5 s | Reject stale required error-register/code reads |
| `feedback_request_period` | 0.02 s | Feedback polling timer |
| `diagnostic_sdo_period` | 1.0 s | Error and raw ELMO safety-register polling |
| `traction_pdo_period` | 0.02 s | Traction commands and CANopen SYNC |

Management/feedback guards run on a steady-clock wall timer at 20 ms.
Status is polled in all drive states at default intervals of at most 0.1 s;
fault reads continue while disabled/faulted. Polling/PDO gaps must remain shorter
than their freshness timeouts.

A module's `feedback_fresh` is true only if **both** drives have fresh velocity,
status and error-register samples, plus steering position and traction error code.
Heartbeats, unrelated SDOs and current-only frames do not renew those fields.
Malformed or wrong-object/subindex replies are not treated as valid feedback.

When any required feedback is missing/stale, a drive is disabled or a known
fault exists, the driver blocks motion and requests stop on **all units**.
It retains polling/SYNC for recovery, without changing the requested safety/battery
enable policy. Recovery clears old targets and requires newly timestamped motion.
Supervisor-loss re-enable also zeros traction targets and steering profile speed.

The wheel-status array header is report time, not CAN acquisition time.
Cached fault values are last-known data; `feedback_fresh=false` must inhibit motion.
`enabled` is false when its required status is stale. Unknown control mode/current
are reported as unknown/NaN rather than fabricated healthy values.

`joint_states` is published only for a complete fresh required feedback set.
Its header is the oldest required motion sample time; optional effort is omitted
when any current sample is unavailable/stale. CAN transport acquisition timestamps
contribute to age; unstamped frames fall back to reception time and cannot reveal
replayed unstamped payloads. See [commissioning tests](../../docs/hardware_monitoring.md).

Timestamped CAN feedback is ordered **per drive and per decoded field** using the
exact original acquisition timestamp, not a ROS-to-steady conversion approximation.
Duplicate/older samples cannot overwrite values, renew freshness, increment motion
generations or trigger initialization side effects. PDO and SDO sources share each
field's watermark: an older PDO status cannot undo a newer SDO disable/fault,
while genuinely newer velocity from that same PDO can still be accepted.
Ordering covers status, position, velocity, current, fault register/code and
control mode. Consuming/resetting a joint-state batch does not reset watermarks.
Zero stamps retain a monotonic local-time fallback; this is compatibility, not
reliable detection of replayed unstamped data. Keep the acquisition-stamping CAN
bridge enabled for the hardware profile.

## Hardware configuration and telemetry

The integrated driver configuration is
[`platform.yaml`](../mobotic_config/config/platform.yaml) and
[`driver.yaml`](../mobotic_config/config/driver.yaml).
The current bindings are:

| Module | Steering | Traction |
|:--|:--|:--|
| front_left | `front_left_steering`, CAN 0x01 | `front_left_traction`, CAN 0x02 |
| front_right | `front_right_steering`, CAN 0x03 | `front_right_traction`, CAN 0x04 |
| rear_left | `rear_left_steering`, CAN 0x07 | `rear_left_traction`, CAN 0x08 |
| rear_right | `rear_right_steering`, CAN 0x05 | `rear_right_traction`, CAN 0x06 |

The four-wheel profile uses `can_heartbeat_period=0.01` s, with the reference
traction ceiling `traction_max_profile_velocity=116053` motor increments/s
and matching per-drive traction limits. With 4096 motor counts/revolution and
16:1 gearing this is ~11.126 wheel rad/s (~3.616 m/s at radius 0.325 m), not a
normal driving-speed request. Kinematics and direct-autonomy limits remain
11.12 rad/s; the supervisor body-speed limit remains 1.25 m/s.

`traction_profile_accel` and `traction_profile_decel` are explicitly 32093
motor increments/s^2, rounded from `1.0 * 16 * 4096 / (2*pi*0.325)` for nominal
1 m/s^2 at the actual wheel radius. The supervisor command ramp remains 0.5 m/s^2.
Driver fallbacks and standalone launch defaults use the same profile; the
standalone launch exposes both acceleration and deceleration overrides.
All three profile parameters must be positive. Recalculate them if wheel radius,
gearing or encoder resolution changes; the driver receives raw controller units.

MoboTerra has four powered steering/traction modules and no casters. CAN addresses
come from the supplied four-module `terraspiro.launch.py` reference and still
require verification on the actual platform. Feedback/stop gates cover all eight
actuators. Old two-module `front_*`/`rear_*` joint names are no longer the configured
public interface; external clients must use the eight names above.

All `drives.*` arrays must have equal lengths; the three `modules.*` arrays must
match one another. Empty module arrays allow suffix-based discovery.
`drives.names` are joint names, not joint `frame_id` values.
Keep the supervisor's `direct_steering_encoder_resolutions` aligned with the
steering entries in `drives.resolutions`. The driver always independently checks
its own configured resolution for both motion inputs.

With `traction_pdo_enabled` and `telemetry_pdo_enabled`, the backend configures
traction status/velocity and steering position/velocity PDOs, plus telemetry
mappings. Only implemented decoded fields are reflected in ROS feedback; configured
temperature/voltage mappings do not imply a recorder or exported temperature/voltage
interface. No JSON/CSV telemetry recorder is included in this package.

Read-only ELMO `0x2086:0` STO and `0x60FD:0` digital inputs are polled even
when disabled. Diagnostics expose raw values, freshness and unsupported/abort
states. The STO bit mapping is unknown; these values are not decoded into
verified active/inactive STO. Current scaling still requires hardware validation.
Raw read/abort replies are ordered per register too. An older abort cannot erase
a newer value, and an older read cannot restore availability after a newer abort.
Their freshness uses backdated acquisition age rather than granting delayed
replies a full new lifetime. These are still monitoring values, not a STO command
or a decoded safety-rated confirmation.

The `/can_errors` diagnostic entry counts `can_rx` frames marked `is_error` and
warns for five seconds after receipt. Bringup enables SocketCAN error reception
with `filters=0:0,#1FFFFFFF`; independently launched CAN bridges need equivalent
configuration. Error frames never refresh drive telemetry or become commands.
Zero counted errors means no error frames observed, not proof of a healthy bus.

## Virtual drive protocol and limitations

`virtual_mobotic_drive` selects `drive_type=steering` (MiControl) or `traction`
(ELMO). It supports the production driver's expedited SDO reads/writes and
acknowledgements, 16-bit enable/controlword writes, mode/error/status objects,
fault clearing, PDO mapping, NMT start/pre-operational/reset and synchronous
RPDO/TPDO layouts. Feedback remains readable while disabled. Disable clears
motion and pending RPDO targets; re-enable does not revive old commands.
Unsupported objects/subindices and wrong write widths receive abort replies.
Raw ELMO STO/digital-input objects are intentionally unsupported, not fabricated
as healthy. Fault injection is available to protocol unit tests, not a ROS service.

Steering position advances toward its target using commanded motor-rpm profile
speed, `encoder_resolution` (4096 ticks/revolution) and `gear_ratio` (121).
Traction velocity feedback follows its commanded encoder-increments/second value
immediately; traction gearing defaults to 16. Current-command feedback follows
the repository's existing `0x3200` convention, not a full ELMO current controller
model. Synthetic current, voltage and temperature are not motor measurements.
Profile acceleration/limit objects are stored but not used for traction dynamics.
There is no inertia, load, torque/brake/STO model, slip or complete CANopen/DS402
state-machine conformance. Legacy steering velocity/current modes do not integrate
position. The 10 ms motion timer uses ROS elapsed time, capped at 0.1 s per update;
paused/backward time does not advance motion.

Use [virtual bringup](../mobotic_bringup/README.md) for the whole stack: CAN topics
are locally reversed and PDO settings are no longer disabled. Standalone launch
also reverses CAN topics and exposes role/scaling arguments; for traction specify
`drive_type:=traction gear_ratio:=16.0`. IDs/roles/scaling in virtual bringup match
the supplied driver YAML; custom hardware configuration needs matching simulator
settings. Mock readiness does not bypass drive feedback gates. This is **not** a
hardware-equivalent simulator or proof of a working ROS end-to-end control path;
the full ROS build and runtime integration test remain required.

A watchdog inside this driver cannot handle the driver's own crash, OS loss or
a disconnected CAN drive. Verify the independent hardware stop/STO/brake chain.

## CAN regression tests

`test_can_feedback_ordering` exercises the production decoder and ordering helper
and is registered with ament when `BUILD_TESTING` is enabled. ROS builds compile
it against real `can_msgs`; run `colcon test --packages-select mobotic_driver`.
For the local ROS-free compiler check only, a minimal message-shaped adapter lives
in `test/standalone/include`. It is never included by ROS build targets. This check
does not validate ROS serialization, subscriptions, the full driver node or hardware.

`test_steering_target_limits` uses the same build/adapter arrangement and exercises
signed tick boundaries, half-away-from-zero rounding, encoder scaling, huge and
non-finite targets, backend rejection and preservation of explicit tick limits.

`test_virtual_drive_protocol` exercises real production request generators,
decoders and readiness checks against the ROS-free simulator core. It covers all
eight drives, initialization, enable/disable, motion, SYNC/PDO configuration,
fault reset, pending-target cancellation, malformed frames and unsupported safety
objects. The same test-only CAN adapter is used for local compiler checks; ament
builds use real `can_msgs`.
