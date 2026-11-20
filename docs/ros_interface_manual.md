# MoboTerra ROS interface reference

This reference applies to the current four-module miControl stack launched by
`scripts/start_MoboTerra` and its current Docker deployment.
Names below are relative to `/moboterra`. Check the installed message definitions
before integrating an external application; updating source does not update an
already-built Docker image.

Use [the customer manual](customer_manual.md) for operation and
[the command reference](commands.md#enter-the-current-ros-environment) to enter
the correctly sourced container environment.

## Ownership and command routing

```text
operator/cmd_vel or autonomy/cmd_vel
                |
                v
            supervisor
                | vehicle/cmd_vel
                v
        ROMB and adapter <----- joint_states ----- driver
                |
                | kinematics/joint_setpoints
                v
            supervisor <----- autonomy/joint_setpoints in mode 2
                |
                | wheel_modules/command
                v
              driver ---- CAN ---- miControl drives
```

The supervisor is the sole final motion authority in this supervised stack.
It also owns `wheel_modules/operation_command` for enable, disable and reset
transactions. External applications publish only to the intended candidate
inputs. They must not publish to internal kinematics, final-command or management
topics. Deployment-specific DDS permissions remain necessary to enforce access
between independently running applications.

Measured `joint_states` feed ROMB direct kinematics, which publishes
`vehicle/measured_velocity`. Wheel odometry integrates that measured velocity
and publishes `vehicle/odom` and the odometry transform. Commanded velocity is
not an odometry measurement.

## Public topic reference

| Relative topic | Message type | Purpose |
| --- | --- | --- |
| `operator/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Operator body-velocity candidate |
| `operator/state` | `mobotic_interfaces/msg/OperatorState` | Connection, physical/source inputs, neutral and known STOP state |
| `operator/mode_request` | `mobotic_interfaces/msg/VehicleModeRequest` | Operator request for mode 0 or 1 |
| `autonomy/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Application body-velocity candidate in AUTONOMOUS |
| `autonomy/joint_setpoints` | `sensor_msgs/msg/JointState` | Application steering/current candidate in KINEMATICOFF |
| `vehicle/mode` | `mobotic_interfaces/msg/VehicleModeState` | Authoritative effective mode, readiness and recovery |
| `joint_states` | `sensor_msgs/msg/JointState` | Complete measured steering and traction joint values |
| `wheel_modules/status` | `mobotic_interfaces/msg/WheelModuleStatusArray` | Four named module records with per-drive status |
| `vehicle/measured_velocity` | `geometry_msgs/msg/TwistStamped` | Measured body velocity |
| `vehicle/odom` | `nav_msgs/msg/Odometry` | Wheel-derived pose, velocity and covariance |
| `battery/system_state` | `mobotic_interfaces/msg/BatterySystemState` | Aggregate pack readiness, HV, faults and minimum SOC |
| `battery/<id>/state` | `sensor_msgs/msg/BatteryState` | Individual pack measurements for `battery1` and `battery2` |
| `battery/state` | `sensor_msgs/msg/BatteryState` | Alias for the primary pack; not the two-pack aggregate |
| `scanner/<location>/scan` | `sensor_msgs/msg/LaserScan` | Laser ranges; locations are `front_left` and `rear_right` |
| `scanner/<location>/safety_zone_status` | `mobotic_interfaces/msg/ScannerSafetyState` | Direct scanner evaluation-path information and validity |
| `safety/state` | `mobotic_interfaces/msg/SafetyState` | FlexiSoft monitoring and derived software safety state |
| `safety/io_state` | `mobotic_interfaces/msg/SafetyIOState` | Mapped safety I/O and known flags |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Component condition and fault details |
| `tf`, `tf_static` | `tf2_msgs/msg/TFMessage` | Namespaced dynamic and static transforms |

## Internal topic reference

| Relative topic | Message type | Publisher and consumer |
| --- | --- | --- |
| `vehicle/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Supervisor to ROMB adapter |
| `kinematics/joint_setpoints` | `sensor_msgs/msg/JointState` | Kinematics candidate back to supervisor |
| `wheel_modules/command` | `mobotic_interfaces/msg/WheelModuleCommandArray` | Supervisor final motion array to driver |
| `wheel_modules/operation_command` | `mobotic_interfaces/msg/WheelModuleOperationCommand` | Supervisor drive-management transaction to driver |
| `internal/component_health` | `mobotic_interfaces/msg/ComponentHealthArray` | Required component freshness and health to supervisor |
| `can_rx`, `can_tx` | `can_msgs/msg/Frame` | Drive/shared CAN receive and transmit |
| `battery_can/can_rx` | `can_msgs/msg/Frame` | Battery monitor's receive-only ROS transport on the shared physical `can0` |

The battery application is receive-only. That does not make the shared physical
CAN link listen-only; the drive and operator paths need normal transmission.
The current deployment does not configure `can1`.

## Modes and transition behavior

| Value | Mode | Meaning |
| --- | --- | --- |
| 0 | MANUAL | Selects the operator path with the configured manual steering policy |
| 1 | AUTONOMOUS | Selects body velocity from `autonomy/cmd_vel` through kinematics |
| 2 | KINEMATICOFF | Selects steering position and traction current from `autonomy/joint_setpoints` |
| 3 | SAFETY | Automatic safety stop state; not selectable |
| 4 | ERROR | Automatic latched fault state; not selectable |
| 255 | UNKNOWN | Initial readiness or monitoring not established |

`selected_mode` retains the operating source; `requested_mode` identifies a
pending transition. `current_mode` includes effective SAFETY, ERROR and UNKNOWN.
ERROR takes display precedence over SAFETY, while `safety_stop_active` still
reports an overlapping stop. Do not interpret the mode number alone as permission
to move. Read `readiness_known`, `ready_to_drive`, `drives_enabled`,
`motion_permitted`, `transition_in_progress`, `error_active` and `reason_codes`.

The supervisor starts with drives disabled and waits for a new B2 enable request.
External readiness returning alone does not enable drives. B2 uses
`wheel_modules/enable_drives`; while disabled it requests supervised recovery and
initialization. While enabled or initializing, another B2 press does not toggle
off or cancel the request. Steering can move during nearest-home alignment even
though traction remains zero. See [drive startup](drive_startup.md).

Mode transitions stop the outgoing source, discard buffered commands and require
fresh body and all-joint standstill, normally sustained for 0.25 seconds. The
configured transition deadline is 10 seconds. A timeout latches a fault. A newly
selected source needs new valid commands after the authority boundary. An
already-running external publisher can provide those new commands and cause
motion when readiness permits.

RP1/U5 high requests MANUAL and low requests AUTONOMOUS. The first complete
receiver sample primes the switch without issuing a mode request. An unchanged
switch does not repeatedly override a service-selected mode. Both autonomous
modes require a fresh valid RP2 Safety Enabled selector in the current profile.

### Autonomy steering selection

`/moboterra/set_steering` (`mobotic_interfaces/srv/SetSteeringMode`) selects
the steering policy for AUTONOMOUS `autonomy/cmd_vel` commands:

| `mode` | Selection | Accepted body velocities |
| --- | --- | --- |
| 0 | STANDARD | Combined `linear.x`, `linear.y` and `angular.z`; startup default |
| 1 | CRAB | Forward, sideways or diagonal translation; `angular.z = 0` |
| 2 | ACKERMANN | Forward/reverse with turning; `linear.y = 0`, bounded curvature, no rotation in place |

ACKERMANN requires `abs(angular.z) <= abs(linear.x) * max_curvature`, where
`max_curvature = 1 / max(abs(wheel_x) + abs(wheel_y))` over the configured wheel
positions, in 1/m. This is the same geometry-derived curvature envelope as manual
Ackermann steering. Input checks allow an absolute numerical tolerance of `1e-9`;
the outgoing velocity still has zero forbidden components and respects the
curvature bound during acceleration, reversing and deceleration.
ACKERMANN uses coupled forward/yaw ramps and passes through zero commanded
velocity before reversing direction, preserving both acceleration limits and
the curvature envelope.

The service request contains `uint8 mode`. The response contains `accepted`,
`transition_started`, `active_mode` and `message`. An accepted change stops the
outgoing command, verifies fresh body and all-joint standstill for 0.25 seconds,
then commits the selection. Until completion, `active_mode` remains the previous
selection. The existing 10-second transition deadline applies; a timeout retains
the previous selection and latches a fault. Commands and pending kinematics
results from before completion are discarded. New valid commands are required
afterward; an already-running publisher can supply them and cause motion.

Observe `vehicle/mode.steering_mode`, `requested_steering_mode`, and
`steering_transition_in_progress`; `transition_in_progress` covers either a
vehicle-source change or a steering change. Repeating the active selection is a
no-op; repeating the pending selection reports it in progress. Conflicting
requests and requests during fault recovery are rejected. Selection is also
available in MANUAL after fresh measured standstill, using the same transition
sequence to preselect the next autonomous policy. It does not change manual
joystick steering. KINEMATICOFF rejects steering-selection requests.

An incompatible autonomous command revokes the previous candidate and pending
motion results and requests a controlled stop. It is not silently converted into
another path. `AUTONOMY_STEERING_COMMAND_REJECTED` appears in `reason_codes`, with
the specific cause in `status_message`. This is a command inhibition, not a
latched drive fault; a fresh compatible command clears it. Safety/readiness loss
continues to use the existing stop and drive-enable policy.

Selection remains across vehicle-mode changes and recovery, and returns to
STANDARD on supervisor restart. It does not enable drives or select AUTONOMOUS.
All safety, RP2, speed, acceleration, feedback and command-freshness checks remain.
Rebuild the image/native workspace and applications using `VehicleModeState` for
the new service and extended status message before deployment; see
[installation and updates](commands.md#installation-and-updates).

## Candidate command contracts

Body-velocity candidates use a valid nonzero current source timestamp and the
configured body frame, `base_link`. Linear velocities are m/s and yaw rate is
rad/s. The supervisor does not transform vectors from `map` or `odom`. Empty,
incorrect, future or expired frames/stamps and non-finite values are rejected.
Operator/autonomy candidate expiry is 0.25 seconds in the current configuration.

KINEMATICOFF input is a stamped `JointState` containing each of the eight joint
names exactly once, with complete finite position, velocity and effort arrays:

| Joint family | Position | Velocity | Effort |
| --- | --- | --- | --- |
| Steering | Target angle in rad | Nonnegative profile speed in rad/s | Zero |
| Traction | Finite placeholder; not a traction position command | Zero | Signed motor current in A |

This mode bypasses inverse kinematics for commands. Measured feedback and
supervision remain active. Native traction mode changes require measured
standstill and verified controller readback. MANUAL/AUTONOMOUS use miControl
velocity mode; KINEMATICOFF uses its current mode. Reduced-speed warnings inhibit
current-mode motion rather than treating amperes as a speed. Zero current is not
proof of physical stopping.

## Final command and measured feedback

`WheelModuleCommandArray` includes `header`, `source_stamp`,
`supervisor_instance`, `authority_generation`, `selected_mode`, `purpose` and
one record for each configured module. The driver independently checks authority,
mode, freshness, completeness and limits.

| Module field | Meaning |
| --- | --- |
| `name` | Configured module name |
| `steering_position` | Target in controller steering coordinates, rad |
| `steering_profile_velocity` | Nonnegative profile speed, rad/s |
| `traction_velocity` | Wheel-contact speed, m/s, in velocity modes |
| `traction_current_control` | True only for current-mode traction commands |
| `traction_current` | Motor current, A; zero in velocity modes |

`purpose` distinguishes MOTION, CONTROLLED_STOP and HOLD. Enable and reset
remain separate management operations. A HOLD or zero command does not replace
the independent hardware stop.

Measured `joint_states` contains all eight joints. Position is radians for
steering and traction rotation. Steering velocity is rad/s; traction velocity
is wheel-contact m/s. `effort` is empty. Use names to associate array values;
do not assume a transport array order.

`wheel_modules/status` contains separate steering and traction `DriveStatus`
records, including actual current in amperes and `current_known`. Native state,
mode and fault values have their own known/freshness information. Module flags
include `startup_pending`, `startup_complete`, `startup_failed`, `position_valid`,
`feedback_fresh`, `enabled` and reset acknowledgement. A last-known zero fault
code or `enabled=false` alone is not fresh health or standstill proof.

## Public services

| Relative service | Type | Purpose |
| --- | --- | --- |
| `vehicle/set_mode` | `mobotic_interfaces/srv/SetVehicleMode` | Request 0, 1 or 2 |
| `set_steering` | `mobotic_interfaces/srv/SetSteeringMode` | Autonomy steering: 0 STANDARD (default), 1 CRAB, 2 ACKERMANN |
| `wheel_modules/enable_drives` | `std_srvs/srv/Trigger` | B2-equivalent initialize/enable request |
| `wheel_modules/clear_errors` | `std_srvs/srv/Trigger` | Separate supervised fault-recovery transaction |
| `vehicle/reset_odometry` | `std_srvs/srv/Trigger` | Reset local wheel odometry with measured standstill |

The legacy `wheel_modules/toggle_drives` remains implemented for service
compatibility; it is not the PAIL B2 behavior. Driver initialization and disable
services under `internal/driver` are not customer operating controls.

Accepted service requests are not completed operations. Track transition or
reset fields on `vehicle/mode`. Recovery requires corrected external blockers,
neutral controls when applicable, valid admission evidence and fresh matching
driver results. The current reset deadline is 20 seconds and driver startup
deadline is 15 seconds. Failed recovery retains inhibition and requires another
explicit request. Old motion commands are discarded.

## Current sensor and operator policies

The current profile marks its PAIL mapping verified. It has no held deadman and
sets `require_neutral_rearm: false`; neutral controls are still required for B2
admission. `require_enable_interlock: false` does not establish a universal
all-mode operator interlock. MANUAL needs valid operator input; the configured
RP2 selector still gates both autonomous modes. Known physical STOP and the
independent safety wiring must be tested on the delivered vehicle.

With Safety Enabled, direct scanner evaluation-path data must be fresh and
valid. Warning fields impose a speed cap of at most 0.3 m/s; protective fields
inhibit motion. Safety Disabled permits only restricted MANUAL software scanner
bypass at the reduced speed. Stale or invalid selector input cannot grant that
bypass. LaserScan alone does not establish protective-field status.

FlexiSoft Ethernet is monitoring-only with `safety.telemetry_required: false`.
Its Ethernet messages do not supply an additional supervisor control gate in
that role. The physical stop/STO circuit operates independently. If this setting
is changed to true, the additional software telemetry gates must be commissioned.

Battery SOC below 20% is a warning, not a drive-enable or motion block. Invalid,
missing or stale battery state, HV disconnection, applicable charging conditions
and battery faults remain blockers. Receiver error-register reports are advisory
under the current profile; invalid or stale input and applicable STOP/link checks
remain separate conditions. Review these settings explicitly at handover.

## Visualization and limitations

`tf` and `tf_static` are namespaced. Use the RViz command in
[the command reference](commands.md#native-maintenance-and-rviz) to set the
namespace and TF remappings. Do not add another `robot_state_publisher` for the
same robot. Odometry is wheel-derived local estimation; it is not a map-based
localization system or proof of actual zero movement.

ROS monitoring is not safety-rated. Message checks, a healthy Docker state and
software watchdogs do not prove hardware stop performance. Validate installed
units, direction, braking, field assignment, timing and loss/recovery behavior on
the actual machine. See [validation and handover](software_readiness.md).
