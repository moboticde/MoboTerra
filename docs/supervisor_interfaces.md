# MoboTerra supervisor interface

The supervisor contains the vehicle mode state machine and remains the single
final motion authority. The implemented update exposes `operator/state`,
`operator/cmd_vel`, `operator/mode_request` and `vehicle/mode`.

Only MANUAL=0, AUTONOMOUS=1 and KINEMATICOFF=2 are selectable. SAFETY=3 and
ERROR=4 are automatic states; UNKNOWN=255 represents initial unknown readiness.
ERROR takes precedence while preserving the overlapping safety-stop flag.

The final motion type on `wheel_modules/command` is now
`mobotic_interfaces/msg/WheelModuleCommandArray`. Internal enable/reset
transactions use `wheel_modules/operation_command`. Applications must not
publish to either internal path or bypass arbitration through kinematics.

KINEMATICOFF commands steering position/profile speed and traction motor current.
Its JointState input uses full finite arrays: traction effort is current [A],
traction velocity is zero and steering effort is zero. Final module records
set `traction_current_control=true`, carry `traction_current` [A] and zero
`traction_velocity`. Other modes use velocity with zero current. The driver
rejects a control flag/target that disagrees with the selected vehicle mode.
Only after fresh sustained standstill does the management command assert
`reconfigure_traction_mode`; the supervisor waits for native mode feedback before
completing selection. This implementation verifies miControl current mode 2 /
`0x3200`. ELMO current selection is rejected until its mapping is verified.
Current mode is inhibited under reduced-speed warnings rather than scaling
amperes as velocity. See [the current-mode contract](../src/mobotic_supervisor/README.md).

Public services are `vehicle/set_mode`, supervisor-owned
`wheel_modules/enable_drives` (B2, like Logitech X), `wheel_modules/toggle_drives`
(explicit legacy toggle), `wheel_modules/clear_errors`, and `vehicle/reset_odometry`.
Reset acceptance is not completion: observe `vehicle/mode` reset fields.
Fresh individually confirmed disabled drives can admit fault clearing when
motion feedback is unavailable. Zero-target enable follows fresh fault-clear
acknowledgements and external/neutral readiness; new sustained standstill and
healthy enabled feedback are required before clearing the supervisor latch.

The supervisor starts with drives disabled and waits for B2. Any blocking
readiness loss withdraws enable and restores this wait. It keeps running,
publishes the cause and operator remedy in `vehicle/mode.status_message`, and
sets `B2_ENABLE_REQUIRED` in `reason_codes`. Returning telemetry does not clear
this wait. A new B2 press after correction and with neutral controls requests
recovery without restarting the program. See [operator recovery](drive_startup.md).

Physical enable permission is separate from motion readiness. A fresh B2
transaction plus external safety/battery/operator/host checks authorize EC-style initialization and
zero-target enable; position/velocity and kinematics feedback are acquired after
enable and remain mandatory for motion. Module status exposes `startup_pending`,
`startup_complete` and `startup_failed`. Startup has a 15 s driver deadline and
is cancelled on lost permission. During an acknowledged reset, the supervisor
may authorize zero-target re-enable to reacquire feedback while ordinary motion
and fault-latch clearance remain blocked until recovery is verified.

FlexiSoft Ethernet feedback has an explicit role in the shared profile.
`safety.telemetry_required: false` selects monitoring only (the four-module
default); the physical safety controller operates ELMO STO independently.
Ethernet safety messages do not affect supervisor control in that role. Direct
scanner fields, operator STOP, battery/drive readiness, known active drive STO
and watchdogs retain their checks. `true` selects the additional FlexiSoft
software interlocks; older profiles omitting this field retain that behavior.

See the [complete ROS interface manual](ros_interface_manual.md) for message
fields, feedback units, mode transitions, recovery, scanner data and hardware
limitations, and the [command reference](commands.md) for runnable examples.

Rebuild the workspace/images after message changes and check the installed
definitions before use. The [software readiness review](software_readiness.md)
records source-level validation and the remaining target-machine checks.
