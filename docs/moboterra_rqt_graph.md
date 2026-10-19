# MoboTerra updated ROS structure

The supervisor contains the mode state machine. This is a source-defined
structure, not a captured live rqt graph.

```text
operator/cmd_vel ----> supervisor ----> vehicle/cmd_vel
autonomy/cmd_vel ---->      ^                 |
                           |                 v
                           +----- kinematics/joint_setpoints
                           |
autonomy/joint_setpoints --> supervisor
                                  |
                         wheel_modules/command
                                  |
                                driver
                                  |
                             joint_states
                                  |
                              kinematics
                                  |
                       vehicle/measured_velocity
                                  |
                               odometry
                                  |
                             vehicle/odom

operator provider --> operator/state --> supervisor and application
supervisor ---------> vehicle/mode ----> driver, operator and application
safety, battery and wheel status ------> supervisor
scanner fields and LaserScan ---------> application and autonomy
```

MANUAL selects operator velocity. AUTONOMOUS selects autonomy velocity.
KINEMATICOFF selects autonomy joint candidates and ignores velocity candidates.
Both paths return to the supervisor before one final wheel-motion publication.
Internal management is separate on `wheel_modules/operation_command`.

In the updated launch, robot_state_publisher is included by default through
`start_description=true`; do not launch a duplicate. Scanner locations are
`front_left` and `rear_right`. All names above are namespace-relative,
normally beneath `/moboterra`.

## Capture the running graph

On the Ubuntu host desktop, after verifying that the running image contains
the updated messages:

```bash
moboterra-gui rqt_graph
```

In a sourced container shell:

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 topic info --verbose /moboterra/wheel_modules/command
ros2 node info /moboterra/mobotic_driver
```

Refresh rqt_graph and show internal/leaf topics. Confirm that the supervisor is
the only final-motion publisher and the driver has no direct kinematics joint
candidate subscription. Services are not shown by rqt_graph. A virtual capture
does not validate SocketCAN or physical safety behavior.

The newer implementation remains in the saved local update rather than the
active pulled checkout. See the [source audit](interface_source_audit.md),
[ROS manual](ros_interface_manual.md) and [command reference](commands.md).
