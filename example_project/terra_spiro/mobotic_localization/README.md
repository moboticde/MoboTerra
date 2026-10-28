# mobotic_localization

SLAM and localization for the Mobotic robot using `slam_toolbox`.

## Contents

- `odom_to_tf` node — republishes `/mobotic/odometry` as `odom → base_link` TF
- `laser_tf_calibrator` node — solves for `base_link → front_scan` from SLAM + odometry
- Launch files:
  - `mapping.launch.py` — build a map using odometry + scan matching
  - `mapping_scan_only.launch.py` — build a map using scan matching only (no odometry)
  - `localization.launch.py` — localize in a saved map
  - `calibrate_laser_tf.launch.py` — estimate the laser transform
- Configs in `config/` and saved maps in `maps/`

## Build

```bash
cd /workspace/cyclonedds_ws
colcon build --packages-select mobotic_localization
source install/setup.bash
```

DDS: this workspace uses CycloneDDS. Make sure the environment has:
(Ja sam imao u dockeru nekih problema sa nekim librarijim, potencijalo se moze koristiti i rmw_fastrtps_cpp)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 1. Map the environment

Launch mapping, then play (or stream live) the scan + odometry topics.

```bash
# terminal 1 — mapping
ros2 launch mobotic_localization mapping.launch.py use_sim_time:=True

# terminal 2 — data source (bag or live robot)
ros2 bag play /workspace/bags/laser_bags/laser-data-collection-1-loc --clock
```

Drop `use_sim_time:=True` and `--clock` when running on a live robot.

Topics to expect:
- `/map` — occupancy grid (for visualization in RViz)
- `/tf` — `map → odom → base_link → front_scan`

### Saving the map

While mapping is still running, serialize the pose graph to disk:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/workspace/cyclonedds_ws/src/mobotic_localization/maps/map'}"
```

This produces `map.data` and `map.posegraph` in `maps/`. Rebuild once so the
installed share directory picks them up:

```bash
cd /workspace/cyclonedds_ws && colcon build --packages-select mobotic_localization
source install/setup.bash
```

Optional: also save a classic `.pgm` + `.yaml` pair for RViz/human inspection
(not required for localization):

```bash
ros2 service call /slam_toolbox/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: '/map', map_url: '/workspace/cyclonedds_ws/src/mobotic_localization/maps/map'}"
```

## 2. Localize in a saved map

```bash
ros2 launch mobotic_localization localization.launch.py use_sim_time:=True

# play a bag (or drive the robot) in another terminal
ros2 bag play /workspace/bags/laser_bags/laser-data-collection-1-loc --clock
```

The launch loads `maps/map.{data,posegraph}` by default. Override with:

```bash
ros2 launch mobotic_localization localization.launch.py \
  map_file_name:=/absolute/path/to/maps
```

### Setting the initial pose

`map_start_at_dock` is `false` in [slam_localization.yaml](config/slam_localization.yaml),
so slam_toolbox will **not** assume the robot is at the map origin. Set the
starting pose with RViz's **2D Pose Estimate** tool, which publishes to
`/initialpose`. Alternatively, set a fixed start pose in the yaml:

```yaml
map_start_pose: [x, y, yaw]
```

## 3. Calibrate the laser transform

If scans don't line up with the map during rotation, the `base_link → front_scan`
static transform is probably off. The calibrator runs a second slam_toolbox
instance with `base_frame == odom_frame == front_scan` so its `map → front_scan`
is pure scan matching, then solves `AX = XB` against `/mobotic/odometry`.

```bash
# terminal 1
ros2 launch mobotic_localization calibrate_laser_tf.launch.py use_sim_time:=True

# terminal 2 — slow playback helps during turns
ros2 bag play /workspace/bags/laser_bags/laser-data-collection-1-loc --clock --rate 0.5
```

Drive (or play a bag that includes) meaningful translation **and** rotation.
Yaw is not observable from straight-line motion alone.

When the bag ends, Ctrl+C the launch. The calibrator prints:

```
Calibrated base_link -> front_scan:
  x:   ... m
  y:   ... m
  yaw: ... rad  (... deg)
  translation fit RMSE: ...
  rotation  consistency RMSE: ...
```

Plug the values into the launch args (`laser_x`, `laser_y`, `laser_yaw`) — see
the defaults in [localization.launch.py](launch/localization.launch.py) and
[mapping.launch.py](launch/mapping.launch.py).

## 4. Scan-only mapping (no odometry)

For data without trustworthy odometry:

```bash
ros2 launch mobotic_localization mapping_scan_only.launch.py use_sim_time:=True
ros2 bag play /workspace/bags/laser_bags/laser-data-collection-1-loc --clock
```

This uses [slam_mapping_scan_only.yaml](config/slam_mapping_scan_only.yaml), which
sets `odom_frame: base_link` and `minimum_travel_*: 0.0` so every scan is fed to
the matcher. Works but is less robust to fast motion than the odom-assisted
mapping.

## Config notes

- **Lagging pose during turns**: caused by `minimum_travel_heading` being too
  large or `minimum_time_interval > 0`. Current configs use `0.1 rad` and `0.0 s`
  respectively.
- **`/map` not published**: slam_toolbox is a lifecycle node. Each launch
  includes a `nav2_lifecycle_manager` that configures + activates it
  automatically. Verify with `ros2 node info /slam_toolbox` — you should see a
  subscription to the scan topic.
- **Message filter dropping scans**: means TF for the scan's timestamp is
  missing. Check the TF chain: `odom → base_link` from `odom_to_tf`, and
  `base_link → front_scan` from the static transform publisher in the launch.

## Topics and frames

| What              | Topic / frame                                   |
|-------------------|-------------------------------------------------|
| Lidar             | `/mobotic/front_scanner/scan` (frame `front_scan`) |
| Odometry          | `/mobotic/odometry`                             |
| Map               | `/map`                                          |
| TF (mapping)      | `map → odom → base_link → front_scan`           |
| TF (scan-only)    | `map → base_link → front_scan`                  |
