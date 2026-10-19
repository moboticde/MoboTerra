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

See the [complete ROS interface manual](ros_interface_manual.md) for message
fields, feedback units, mode transitions, recovery, scanner data and hardware
limitations, and the [command reference](commands.md) for runnable examples.

The saved updated implementation is not yet integrated into the active
checkout after the pull. Check the installed image definitions before use.
The [source audit](interface_source_audit.md) explains the code-state mismatch.
