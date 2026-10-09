# mobotic_odometry

Planar wheel odometry from measured body velocity:
`joint_states -> mobotic_kinematics -> agv_vel -> mobotic_odometry`.
Forward kinematics uses all four steering positions and traction velocities,
the actual module coordinates, mounting yaws and wheel radius. No commanded
velocity, laser scan or autonomous planner is used for integration.

## Interfaces

| Direction | Namespace-relative interface | Type / purpose |
|:--|:--|:--|
| Input | `agv_vel` | `geometry_msgs/msg/TwistStamped`, measured body vx/vy/wz |
| Output | `odometry` | `nav_msgs/msg/Odometry` |
| Output | `tf` | `odom -> base_link`, optional, same pose/stamp as odometry |
| Output | `diagnostics` | Feedback freshness and persistent gap warning |
| Service | `odometry/reset` | `std_srvs/srv/Trigger`; requires fresh measured standstill |

The default namespace is `/moboterra`. Pose is in `odom`; twist is in
`base_link`, following the [Odometry message contract](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Odometry.msg).
The velocity frame must match exactly; no TF conversion or relabeling is performed.
Both launch files remap the broadcaster's `/tf` to namespace-relative `tf`,
matching the standalone robot description. RViz needs the same TF remappings.

## Integration and limitations

The first accepted sample anchors the timeline at `initial_pose` (default zero).
Subsequent samples use acquisition-time intervals and the average of adjacent
measured body twists, with the SE(2) exponential for translation/rotation.
This supports forward/reverse driving, lateral crab motion, pure rotation and
combined curves without the small-angle Euler error of a simple x/y update.
It is exact for constant body twist, approximate when twist changes within a sample.
Yaw is normalized; quaternion output has unit norm. Output z is zero because the
local odom plane is at the initial `base_link` height, not a floor-height correction.
The URDF already places base_link at wheel axle height above the floor.

Only newer, timestamped, finite, plausibly bounded planar samples are accepted.
Freshness is checked against both acquisition time and steady receipt time.
Feedback loss stops odometry/TF publication; no freshly stamped held pose or fake
zero velocity is repeatedly published. Duplicate/older samples cannot move the
estimate or renew freshness. Paused ROS time cannot extend the receipt timeout;
backward time fails closed until catch-up or a deliberate process restart.
Future-clock skew up to 100 ms is tolerated, matching the kinematics cache.

A gap beyond `max_integration_interval` or `feedback_timeout`, or invalid newer
feedback, breaks integration continuity. The first recovery sample holds the
last pose, adds covariance for the unobserved interval and anchors a new timeline.
Subsequent samples resume integration; diagnostics retain WARN until an explicit
reset. Travel during the outage cannot be recovered from these velocities.

Covariance propagates x/y/yaw correlations using pose and twist Jacobians, with
positive twist noise and a per-second process-noise floor. z/roll/pitch are not
measured and carry large variance. Defaults are heuristic commissioning values,
not measured statistical accuracy or a guaranteed error bound. Tune wheel radius,
encoder offsets, slip/noise and gap rates on secured hardware. Wheel odometry
drifts and cannot replace IMU/laser localization; no `map -> odom` is provided.

## Launch and TF ownership

Integrated bringup starts this node by default in hardware and virtual profiles.
Use `start_odometry:=false` to omit it, or `publish_odom_tf:=false` to keep wheel
odometry while an external EKF/localization system owns `odom -> base_link`.
There must be only one broadcaster for that transform. Robot-state publisher
owns `base_link`'s joint descendants, not `odom -> base_link`.
The CAD description is still launched separately.

Standalone, alongside driver/kinematics (do not duplicate integrated bringup):

```bash
ros2 launch mobotic_odometry odometry.launch.py robot_name:=moboterra
ros2 topic info /moboterra/odometry -v
ros2 topic echo /moboterra/diagnostics
```

The node deliberately does not automatically respawn: a restart initializes a new
odometry origin, which downstream localization must explicitly handle. The reset
service also changes the origin to configured `initial_pose`, clears uncertainty,
retains replay watermarks and waits for a new measured sample. Call only during
commissioning with downstream consumers aware of the reset:

```bash
ros2 service call /moboterra/odometry/reset std_srvs/srv/Trigger '{}'
```

Tests exercise the real integration engine, ROS callback/message adapters and
launch/installation contracts. They do not validate DDS, ROS executor timing,
TF listeners, a full ROS build or physical wheel slip.
