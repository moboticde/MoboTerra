# MoboTerra updated ROS structure

The supervisor contains the mode state machine. This is a source-defined
structure, not a captured live rqt graph.

## One connected white-background graph

The current two-module hardware launch is shown as one connected graph, with
each node appearing once. It contains all 15 hardware-stack software nodes and
one optional external application/autonomy node. Topic names label the arrows,
as in the rqt_graph node-only view. Names are relative to `/moboterra`, including
the separate `battery_can/socket_can_receiver` namespace.
Related nodes sit inside six labelled blocks: battery, drive CAN transport,
safety monitoring, control/drive authority, scanners/diagnostics and
feedback/odometry/TF. The three equal upper blocks share one horizontal node
grid. The two equal lower blocks share the same two-above/one-below node grid;
both scanners remain side by side. The control chain and optional application
are centred on the graph. All nodes have equal size and all blocks connect
within one graph.

- [SVG — scalable connected graph](diagrams/moboterra-rqt-graph-connected-white.svg)
- [PNG — white-background connected graph](diagrams/moboterra-rqt-graph-connected-white.png)
- [PDF — printable connected graph](diagrams/moboterra-rqt-graph-connected-white.pdf)

Regenerate with `python docs/diagrams/render_connected_ros_graph.py`
(Matplotlib required). The renderer checks unique node identity, connectivity
and label overlap. Its JSON manifest records nodes, main topic edges and the
omitted secondary monitoring fan-out. Services, leaf/debug topics and /rosout
are omitted. The optional external application represents a command publisher
and telemetry consumer; it is not launched by the hardware stack.

The diagram reflects `platform-front-right-rear-left.yaml`: CAN PAIL, two modules,
two scanners, hardwired Siemens STO monitoring, three SocketCAN transport nodes,
description, odometry and diagnostics. USB joystick, FlexiSoft bridge and virtual
drive processes belong to other profiles and are not launched in this one.
The hardware safety wiring remains independent of ROS.

For short descriptions and message types of the arrows, see the
[main topic reference](ros_interface_manual.md#main-topic-reference).
The [command-path and latency guide](control_path_and_latency.md) explains the
kinematics-to-supervisor-to-driver route, its differences from `romb_ws_ec`,
the final wheel command fields and the configured timing.

Transport node identities were checked against the upstream
[receiver launch](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan/launch/socket_can_receiver.launch.py)
and [sender launch](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan/launch/socket_can_sender.launch.py).
This is a source-derived diagram, not a captured live graph.

## Previous sectioned view

The current PAIL / Ecobox two-module profile is illustrated in aligned command,
feedback, readiness and physical-connection sections. Ellipses denote ROS nodes;
rectangles denote topics. A repeated node name denotes the same node in another
view, not another process. Topic names are relative to `/moboterra`.

- [SVG — scalable graph](diagrams/moboterra-rqt-graph-white.svg)
- [PNG — white-background image](diagrams/moboterra-rqt-graph-white.png)
- [PDF — printable vector graph](diagrams/moboterra-rqt-graph-white.pdf)

Regenerate with `python docs/diagrams/render_ros_graph.py` (Matplotlib required).
This graph is derived from the current source and selected hardware profile;
it does not assert that all nodes or hardware are currently running.

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

The graph reflects the active local source changes. See the [source audit](interface_source_audit.md),
[ROS manual](ros_interface_manual.md) and [command reference](commands.md).
