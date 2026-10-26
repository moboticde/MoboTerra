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

Public services are `vehicle/set_mode`, supervisor-owned
`wheel_modules/clear_errors`, and `vehicle/reset_odometry`.
Reset acceptance is not completion: observe `vehicle/mode` reset fields.
Fresh individually confirmed disabled drives can admit fault clearing when
motion feedback is unavailable. Zero-target enable follows fresh fault-clear
acknowledgements and external/neutral readiness; new sustained standstill and
healthy enabled feedback are required before clearing the supervisor latch.

Physical enable permission is separate from motion readiness. Fresh external
safety/battery/operator/host checks authorize EC-style initialization and
zero-target enable; position/velocity and kinematics feedback are acquired after
enable and remain mandatory for motion. Module status exposes `startup_pending`,
`startup_complete` and `startup_failed`. Startup has a 15 s driver deadline and
is cancelled on lost permission. During an acknowledged reset, the supervisor
may authorize zero-target re-enable to reacquire feedback while ordinary motion
and fault-latch clearance remain blocked until recovery is verified.

See the [complete ROS interface manual](ros_interface_manual.md) for message
fields, feedback units, mode transitions, recovery, scanner data and hardware
limitations, and the [command reference](commands.md) for runnable examples.

Rebuild the workspace/images after message changes and check the installed
definitions before use. The [software readiness review](software_readiness.md)
records source-level validation and the remaining target-machine checks.
