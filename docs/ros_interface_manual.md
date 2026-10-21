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
| `diagnostics` | diagnostic_msgs/msg/DiagnosticArray | Component details for root-cause investigation |

The driver publishes no separate scalar wheel-motion topics. Consumers use the
aggregated samples. Old `manual/*`, `vehicle/mode_state`, `agv_vel` and
`odometry` names are not aliases in this updated implementation.

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
[rad/s]. The driver synchronizes all configured joints within the configured SYNC
window and preserves acquisition time. `effort` is empty; electrical motor
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

Traction position is read through Elmo at startup and cyclically using PDOs.
There is no direct encoder connection/configuration step. The current physical
profile deliberately inhibits enable and authoritative joint publication until
the retained Elmo reference, scale and identity are commissioned. The virtual
launch alone enables the synthetic source. Never set `virtual_position_source`
on physical CAN to bypass this gate. The persistent directory and durable
helper do not yet establish recovery across power loss or unpowered movement.

## Scanner information for autonomy

For `front_left` and `rear_right`, public relative topics are:

- `scanner/<location>/scan` for LaserScan, plus configured native scanner outputs.
- `scanner/<location>/safety_zone_status` for ScannerSafetyState.
- `scanner/<location>/diagnostics` for per-scanner DiagnosticArray.

ScannerSafetyState provides communication, protective/warning clear state, OSSD
state and corresponding known flags. The generic safety adapter derives fields
from the mapped safety-controller I/O; field-set number is currently unknown.
Protective stops and unavailable monitoring are distinguished. LaserScan
presence does not establish a clear protective field or safety permission.

## Service reference

| Relative service | Type | Use |
| --- | --- | --- |
| `vehicle/set_mode` | mobotic_interfaces/srv/SetVehicleMode | Request MANUAL=0, AUTONOMOUS=1 or KINEMATICOFF=2 |
| `wheel_modules/clear_errors` | std_srvs/srv/Trigger | Supervisor-owned asynchronous, standstill-guarded fault reset |
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
