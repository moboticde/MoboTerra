# mobotic_kinematics

Kinematics bridge between supervised vehicle velocity commands and the MoboTerra wheel-module interface.

Configured modules are `front_left`, `front_right`, `rear_left` and `rear_right`,
each with steering and traction. There are no casters. The user-confirmed module
spacing follows actual CAD steering axes: 1.69 m front–rear / 0.99 m left–right,
with the restored four-wheel radius 0.325 m. All mounting yaws use the four-wheel
hardware reference, not the different two-diagonal platform's calibration. Both kinematic directions
use all four modules, and command publication requires all eight feedback joints.

The node subscribes to:

- `cmd_vel` (`geometry_msgs/msg/TwistStamped`)
- `joint_states` (`sensor_msgs/msg/JointState`)

It publishes:

- `kinematics/joint_setpoints` (`sensor_msgs/msg/JointState`)
- `agv_vel` (`geometry_msgs/msg/TwistStamped`)

Inverse kinematics uses the configured module coordinates to compute steering position and traction velocity. It keeps
steering motion within 90 degrees of the measured orientation by reversing wheel direction when appropriate. Traction
targets are scaled together when any target exceeds `max_traction_velocity`, preserving the requested motion direction.
The resulting joint command is sent to the driver's dedicated kinematics input.

Forward kinematics converts measured steering positions and traction velocities back to a least-squares planar body
velocity. `agv_vel` preserves the input `joint_states` timestamp, so the supervisor
cannot mistake republished old feedback for a new standstill sample.
Measured `agv_vel` also feeds the separate
[mobotic_odometry](../mobotic_odometry/README.md) node, which integrates planar
pose and publishes `odometry` and optional `odom -> base_link` TF. It is started
by bringup; kinematics itself still publishes velocity, not pose. The separate
[mobotic_description](../mobotic_description/README.md) package provides the CAD
model, separate CAD steering-unit meshes and standalone joint TF publishing.
The user confirmed CAD floor Z=0 and CAD-aligned module XY; each wheel axle is
at floor + radius. Steering-to-axle vertical offsets do not change planar
kinematics because their XY centres coincide.

The node does not publish commands until complete, fresh joint feedback is available.
`joint_feedback_timeout` defaults to 0.3 s and is checked on every velocity command
against both the original acquisition timestamp and steady-clock receipt age.
Delayed samples get only their remaining acquisition-age budget; paused ROS time
cannot keep cached feedback alive. Zero timestamps and samples more than 100 ms
in the future are rejected. Duplicate/older timestamps cannot overwrite the cache
or extend its lifetime. A malformed newer message invalidates availability;
recovery requires a genuinely newer, valid complete sample. All configured joints
are validated together, and steering positions are committed only after finite
forward kinematics succeeds. Joint order may vary; additional uniquely named
joints are allowed if the position/velocity arrays remain complete and finite.

On feedback loss, no new joint commands are published: the driver's independent
motion watchdog stops the previous targets. No synthetic steering position is
used to construct a stop command from stale feedback.

Untimestamped, stale, future-dated, and
non-finite velocity commands are rejected, allowing the driver's watchdog to stop motion if the upstream command path
fails.

Launch with the supplied MoboTerra geometry:

```bash
ros2 launch mobotic_kinematics mobotic_kinematics.launch.py
```

Review the values in [shared `platform.yaml`](../mobotic_config/config/platform.yaml) against the final mechanical measurements before operating the
vehicle. Set `traction_control_mode` to the same `velocity` or `current` mode configured for the traction drives.

The ROS-independent C++ cache regression executable is registered with the ament
test runner when `BUILD_TESTING` is enabled, including test-result reporting.
Run it through `colcon test --packages-select mobotic_kinematics` on the ROS build host.
These tests do not replace node-level
ROS integration or secured hardware loss/recovery testing.

Node-only timeouts and minimum-speed policy live in `mobotic_config/config/kinematics.yaml`.
Standalone launch accepts `platform_config` and an optional `parameters_file` policy overlay.
