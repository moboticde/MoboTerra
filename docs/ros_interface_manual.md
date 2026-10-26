# MoboTerra operator and vehicle interface manual

MoboTerra accepts either operator velocity, autonomous body velocity or
autonomous wheel setpoints. The supervisor contains the mode state machine,
checks readiness and faults, and publishes the only final wheel-motion command.
The application reads `vehicle/mode` for operating state and `operator/state`
for operator-device inputs. These messages have different responsibilities.

This reference describes the interface now integrated into the active checkout.
An existing installed container may still contain the older implementation.
Check the installed message definitions before use as described in
[the command reference](commands.md#6-inspect-the-running-ros-system).
The [source audit](interface_source_audit.md) records the integration and
hardware release constraints.

## Architecture and ownership

Names below are relative to `/moboterra`, the default robot namespace.
Package and node names do not have to change when public topics are renamed.
Raw joystick input remains `joy`; `operator/state` is semantic state, not a
replacement name for a Joy message.

```text
Operator velocity ─┐
Autonomy velocity ─┼─> Supervisor with vehicle mode state machine
Autonomy joints ───┘             │
                   velocity mode│           KINEMATICOFF
                                v                 │
                           Kinematics             │
                                │ candidate       │
                                └─> Supervisor <──┘
                                         │ final wheel array
                                         v
                                       Driver
                                         │ joint_states
                      Kinematics <───────┤
                           │             └─> robot_state_publisher
                           v
                vehicle/measured_velocity
                           │
                        Odometry ──> vehicle/odom
```

Safety, battery, wheel health and required component monitoring feed the
supervisor. The supervisor publishes `vehicle/mode` to the application, operator
provider and driver. Scanner field status, LaserScan, wheel feedback and odometry
are also available to autonomy. The independent hardware safety chain remains
responsible for physical safety; ROS monitoring is not safety-rated.

## Main topic reference

Types in the following table use ROS 2 `package/msg/Type` notation.

| Relative topic | Message type | Owner and purpose |
| --- | --- | --- |
| `operator/cmd_vel` | geometry_msgs/msg/TwistStamped | Operator provider; candidate body velocity |
| `operator/steering_setpoints` | sensor_msgs/msg/JointState | Virtual PAIL operator; stationary steering request with zero traction, checked by the supervisor |
| `operator/state` | mobotic_interfaces/msg/OperatorState | Operator provider; physical inputs and source validity |
| `operator/mode_request` | mobotic_interfaces/msg/VehicleModeRequest | Operator request for modes 0/1 only |
| `autonomy/cmd_vel` | geometry_msgs/msg/TwistStamped | External autonomy; candidate for AUTONOMOUS |
| `autonomy/joint_setpoints` | sensor_msgs/msg/JointState | External autonomy; candidate for KINEMATICOFF |
| `vehicle/cmd_vel` | geometry_msgs/msg/TwistStamped | Supervisor to kinematics; internal candidate |
| `kinematics/joint_setpoints` | sensor_msgs/msg/JointState | Kinematics back to supervisor; not directly to driver |
| `wheel_modules/command` | mobotic_interfaces/msg/WheelModuleCommandArray | Supervisor only; final motion for all configured modules |
| `wheel_modules/operation_command` | mobotic_interfaces/msg/WheelModuleOperationCommand | Supervisor; internal enable/reset transaction |
| `vehicle/mode` | mobotic_interfaces/msg/VehicleModeState | Supervisor; effective mode, selected mode and readiness |
| `joint_states` | sensor_msgs/msg/JointState | Driver; authoritative measured joint state |
| `wheel_modules/status` | mobotic_interfaces/msg/WheelModuleStatusArray | Driver; steering/traction health and current |
| `vehicle/measured_velocity` | geometry_msgs/msg/TwistStamped | Kinematics; measured body velocity |
| `vehicle/odom` | nav_msgs/msg/Odometry | Odometry; wheel-derived pose and twist |
| `battery/state` | sensor_msgs/msg/BatteryState | Public battery telemetry, SOC as a fraction |
| `battery/system_state` | mobotic_interfaces/msg/BatterySystemState | Battery monitor; aggregate pack readiness/faults |
| `safety/state` | mobotic_interfaces/msg/SafetyState | Safety monitor; software motion permission and stop state |
| `safety/io_state` | mobotic_interfaces/msg/SafetyIOState | Safety monitor; mapped I/O and known flags |
| `can_rx`, `can_tx` | can_msgs/msg/Frame | SocketCAN transport; raw drive/PAIL frames received from or sent to can0 |
| `battery_can/can_rx` | can_msgs/msg/Frame | Separate receive-only battery CAN transport to battery monitor |
| `scanner/<location>/scan` | sensor_msgs/msg/LaserScan | Scanner to application/diagnostics; measured laser ranges |
| `scanner/<location>/safety_zone_status` | mobotic_interfaces/msg/ScannerSafetyState | Scanner adapter to consumers; mapped field/OSSD state with known flags |
| `internal/component_health` | mobotic_interfaces/msg/ComponentHealthArray | Platform diagnostics to supervisor; required component freshness and health |
| `tf`, `tf_static` | tf2_msgs/msg/TFMessage | Odometry/robot state publisher; dynamic and static coordinate transforms |
| `diagnostics` | diagnostic_msgs/msg/DiagnosticArray | Component details for root-cause investigation |

The driver publishes no separate scalar wheel-motion topics. Consumers use the
aggregated samples. Old `manual/*`, `vehicle/mode_state`, `agv_vel` and
`odometry` names are not aliases in this updated implementation.

The driver subscribes to final `wheel_modules/command`, not directly to
`kinematics/joint_setpoints`. The supervisor validates and forwards a correlated
candidate inside its receive callback without another periodic control wait.
See [wheel command fields, EC comparison and latency](control_path_and_latency.md)
for the complete payload, the 20 ms supervisor/PDO periods and why these periods
are not measured end-to-end latency or guaranteed stopping times.

## Operating modes

| Value | Mode | Behavior |
| --- | --- | --- |
| 0 | MANUAL | Selects `operator/cmd_vel`; requires fresh connected and rearmed input |
| 1 | AUTONOMOUS | Selects `autonomy/cmd_vel`; ignores autonomy wheel candidates |
| 2 | KINEMATICOFF | Selects `autonomy/joint_setpoints`; velocity candidates have no motion effect |
| 3 | SAFETY | Automatic confirmed safety-chain stop, never selected by an application |
| 4 | ERROR | Automatic required-system fault; latched until verified explicit reset |
| 255 | UNKNOWN | Startup monitoring/readiness has not yet been established |

Only 0/1/2 are selectable. AUTO_VELOCITY and AUTO_DIRECT remain numeric aliases
for 1 and 2. A mode change blocks candidates, stops through the outgoing path
and checks fresh body and all-joint standstill. Default proof duration is 0.25 s;
limits are 0.02 m/s body translation, 0.02 rad/s yaw and 0.1 rad/s per joint.
A 10 s timeout latches ERROR. Fresh commands are required after a completed
transition; a mode request never starts motion by itself.

`current_mode` is effective state. `selected_mode` retains the intended control
mode during SAFETY/ERROR. ERROR takes precedence if both are present, while
`safety_stop_active` preserves the overlapping safety-stop indication.
Read `error_active`, `reason_codes`, `motion_permitted`, `transition_in_progress`,
`readiness_known`, `ready_to_drive` and `drives_enabled` as well.

### Selecting a mode on the two-module PAIL chassis

The supervisor starts with `selected_mode=MANUAL`, while `current_mode=UNKNOWN`
until startup readiness is established. External readiness starts drive
initialization and zero-target enable automatically; selecting a mode is not a
separate motor-enable operation. In all three selectable modes, Elmo steering
stays in position control and traction stays in velocity control.

The confirmed PAIL mode input is U5 active low. Raw U5 high maps to MANUAL (0);
raw U5 low maps to AUTONOMOUS (1). The receiver's first complete sample after
startup or reconnection only primes the switch position. It does not request a
mode. Change the switch position to send a request, or use the service below.
An unchanged switch cannot continually override a service-selected mode.
The physical labels/left-right orientation should be checked on the actual radio.

With a sourced workspace and the stack running, request a mode using one of:

```bash
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 0}'
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 1}'
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 2}'
```

The service response confirms acceptance, not completion. While changing modes,
`selected_mode` remains the outgoing mode, `requested_mode` holds the target,
`transition_in_progress=true` and `motion_permitted=false`. Buffered motion is
discarded. Velocity modes use their outgoing controlled stop; leaving direct
mode commands zero traction and holds actual steering. Fresh body and all-joint
feedback must prove standstill for 0.25 s before the target becomes selected.
Requesting the already-selected mode also runs this stop/standstill sequence.
Transition alone normally keeps drives enabled, subject to the external gates.
A fault or safety loss can disable them.

After completion, commands must have stamps at or after the new authority
boundary. Manual input additionally returns to neutral; this PAIL has no held
deadman. A connected neutral manual source can report `motion_permitted=true`
while requesting zero velocity. AUTONOMOUS needs an external stamped body
velocity publisher on `autonomy/cmd_vel`. KINEMATICOFF needs complete stamped
wheel candidates on `autonomy/joint_setpoints`; it bypasses inverse kinematics
for commands but retains measured feedback, monitoring and the supervisor's
final authority. Selecting either autonomous mode does not start a planner.
An existing publisher sending new commands can start motion after the switch.

The PAIL connection and known STOP state remain required in autonomous modes
because this profile sets `require_operator_interlock=true`. Operator driving
axes and boost have no motion authority there; mode/reset and STOP monitoring
remain available. Commands expire after 250 ms by default. The driver also
checks the supervisor instance, authority generation and selected mode.

Inspect progress and blockers with:

```bash
ros2 topic echo /moboterra/vehicle/mode
```

A transition that reaches 10 s latches `MODE_TRANSITION_TIMEOUT` immediately.
Further mode requests are refused until explicit reset succeeds. A new command
or another mode request does not clear ERROR. Safety stops preserve the selected
mode; after a non-latched protective stop clears, fresh commands and manual
neutral rearm are required. Any accompanying latched fault still requires reset.

On the Siemens two-module chassis, the physical scanner/controller/STO chain
operates independently of ROS. The supervisor has no Siemens Ethernet input.
It can indicate SAFETY from a known PAIL STOP or known drive STO; a LaserScan
alone does not establish protective-field state. Evaluated scanner output paths
are read separately over Ethernet; both field-status streams are motion
prerequisites. Unknown physical STO telemetry does not prove the chain is clear.

### Reset recovery when disabled drives withhold motion telemetry

B2 and `wheel_modules/clear_errors` accept a recovery request when external
interlocks and neutral controls are ready and either sustained measured
standstill is proven or every initialized drive has a fresh native disabled
state. Known movement rejects the request. An aggregate `enabled=false` with
unknown/stale individual status is insufficient. Missing motion data is never
interpreted as zero velocity.

The driver independently verifies admission and the current supervisor/reset
ID, disables/zeros drives and clears faults once. Fresh fault-clearance
acknowledgements permit zero-target enable while reset and fault latches remain
active. Completion requires enabled, initialized, fault-free modules with valid
fresh feedback, required component health and 0.25 s of new sustained measured
body/all-joint standstill. Pre-reset samples, frozen samples and gaps cannot
satisfy this proof. Only then is the latch cleared; old commands are discarded
and manual neutral rearm is required. Timeout requests disable and retains ERROR.
See [the complete recovery sequence](drive_startup.md).

SAFETY includes confirmed protective stops, physical E-stop and known STO or
safety-enable stops from mapped inputs. Missing monitoring is not proof of a
confirmed safety stop: required monitoring loss becomes ERROR after the startup
grace. Required drive faults, invalid position continuity, required component
health failures and mode-transition timeout also produce ERROR. Not every
diagnostic warning is a latched system error.

## Operator inputs and recovery

`OperatorState` contains `source_id`, `connected`, `command_active`,
`deadman_pressed`, `deadman_state_known`, `estop_pressed`, `estop_state_known`,
`controls_neutral`, `turbo_active` and `status_message`, with the acquisition
stamp in `header`. Timer republication must not make an old input look new.

`estop_pressed` refers to the physical E-stop on the operator device. It is not
the scanner stop, an aggregated vehicle stop or a mode selection. Unknown
inputs must not be interpreted as confirmed released buttons. The default USB
gamepad reports `estop_state_known=false`; REMdevice requires a verified PDO
mapping before its physical controls can be relied on.

After a protective stop clears, a fresh command is required. Manual control
also requires neutral controls; a configured deadman must be released and then
pressed again while neutral. The default USB profile has no deadman button
assigned. System errors require a supervisor reset transaction in addition to
cleared causes. Old motion is not replayed after reset or reconnection.

## Joint commands and actual feedback

The default four modules are `front_left`, `front_right`, `rear_left` and `rear_right`.
Their eight names end in `_steering` and `_traction`, for example
`front_left_steering` and `front_left_traction`. Use message `name` values to
associate arrays; do not assume a positional ordering from an earlier worksheet.
The [two-module profile](two_module_setup.md) selects only `front_right` and
`rear_left`, so its feedback and command contract contains four joints.

For actual `joint_states`, `position` contains steering angles and wheel rotation
angles [rad], and `velocity` contains steering and traction rotational velocities
[rad/s]. The driver requires coherent samples from all configured joints and
preserves acquisition time: physical samples use a bounded acquisition skew;
virtual samples use the configured SYNC window. `effort` is empty; electrical motor
current is not torque and is not placed in JointState.effort.

In KINEMATICOFF, the external JointState candidate must contain every configured
joint exactly once with finite position/velocity arrays and empty effort.
Steering position is a target [rad]; steering velocity is a nonnegative profile
speed [rad/s]; traction velocity is signed [rad/s]. Traction position entries
are not traction-position targets. No public motor-current control is exposed.

All velocity candidates require a valid current source stamp and the configured
body frame, normally `base_link`. Empty/map/odom frame IDs, future/expired
commands, incomplete joints and non-finite values are rejected. A relabeled
map-frame vector is not transformed automatically. The supervisor validates
kinematics candidates and emits one authority-tagged final array; the driver
independently validates it. Applications must not publish on internal command
topics. Production DDS permissions are still needed to enforce publisher roles.

## Wheel health current and retained position

Each `wheel_modules/status` module has separate `steering` and `traction`
DriveStatus records. Use each drive's `motor_current` [A] with `current_known`,
native/status/control mode known flags, fault values and `status_stamp`,
`fault_stamp`, `current_stamp`. Module-level fields include `initialized`,
`position_valid`, `position_reason`, `feedback_fresh`, `enabled`, reset ID and
acknowledgement, plus brake/STO values and known flags.

Elmo actual current 0x6078 is scaled using fresh rated current 0x6075. Missing or
stale rating yields unknown current, not guessed amperes. Missing physical
brake/STO mappings remain unknown. Known flags and freshness must be checked
before a zero fault or a numerical reading is treated as valid.

Physical startup waits for external supervisor readiness, configures controller
modes/PDOs/ramps and enables with zero targets. Motion feedback is acquired after
enable; Elmo 0x6064 is read cyclically through configured TPDO3 or SDO in a non-PDO
profile. Fresh complete feedback is required before motion. There is no direct
encoder connection, reference-zeroing or retained-reference prerequisite. Module
status distinguishes startup_pending, startup_complete and startup_failed.
See [startup and B2 recovery](drive_startup.md); continuity across a controller
coordinate reset or unpowered movement is not inferred.

## Scanner information for autonomy

For `front_left` and `rear_right`, public relative topics are:

- `scanner/<location>/scan` for LaserScan, plus configured native scanner outputs.
- `scanner/<location>/safety_zone_status` for ScannerSafetyState.
- `scanner/<location>/diagnostics` for per-scanner DiagnosticArray.

ScannerSafetyState provides communication, protective/warning clear state, OSSD
state and corresponding known flags. On hardware the adapter reads evaluated
application output paths from each scanner's `raw_data`, with validity checks
and a 0.3 s deadline. Physical OSSD and field-set numbers remain unknown. The
supervisor combines both scanners: red commands zero immediately; yellow ramps
to an absolute 0.3 m/s cap; both clear remove the scanner cap. Missing/unknown
field status blocks motion. This applies to manual, autonomous velocity and
direct wheel commands, including the two-module hardware-STO profile. Virtual
bringup explicitly uses mock safety I/O instead of hardware scanner messages.
Protective stops and unavailable monitoring are distinguished. LaserScan
presence does not establish a clear protective field or safety permission.

## Service reference

| Relative service | Type | Use |
| --- | --- | --- |
| `vehicle/set_mode` | mobotic_interfaces/srv/SetVehicleMode | Request MANUAL=0, AUTONOMOUS=1 or KINEMATICOFF=2 |
| `wheel_modules/clear_errors` | std_srvs/srv/Trigger | Supervisor-owned staged fault recovery; disabled-state admission and fresh standstill before completion |
| `vehicle/reset_odometry` | std_srvs/srv/Trigger | Reset wheel odometry only with fresh measured standstill |
| `internal/driver/initialize_all_drives` | std_srvs/srv/Trigger | Commissioning only; rejected while drives are enabled |
| `internal/driver/disable_all_drives` | std_srvs/srv/Trigger | Internal disable; not a persistent safety override |

For mode selection, inspect `accepted` and `transition_started`, then monitor
`vehicle/mode` for the effective state and completion. The service response's
`current_mode` contains selected mode, not the effective SAFETY/ERROR override.

For fault reset, Trigger success means accepted, not completed. Monitor
`reset_in_progress`, `reset_id`, `reset_result`, `error_active` and reason codes.
The supervisor waits for matching fresh driver acknowledgements, standstill
and cleared root causes. A 5 s timeout stays latched. Resetting odometry does
not reset an encoder or clear a system fault. Reinitialization and automatic
enable retries do not implicitly clear errors.

## Live inspection and release checks

Use the sourced Docker-shell commands and service examples in
[the command reference](commands.md). The updated integrated bringup enables
description publication by default; do not start another robot_state_publisher.
Capture the actual graph with `moboterra-gui rqt_graph` while the target is
running. A source-defined architecture is not a captured live ROS graph.

Verify that only the supervisor publishes `wheel_modules/command`, the driver
does not subscribe directly to kinematics joint candidates, all configured measured
joints are coherent, current units/known flags are correct, and safety/fault
recovery never replays stale motion. Hardware commissioning remains necessary
after source integration and a ROS 2 Jazzy build/test run.
