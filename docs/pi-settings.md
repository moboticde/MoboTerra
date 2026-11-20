# Native environment and project settings

This guide covers optional native engineering maintenance on the existing
Ubuntu robot installation. Customer installation, ROS commands and RViz use
the complete Docker image via `bash ./install.sh` and `moboterra-ctl`; native ROS
and a host ROMB runtime are not customer prerequisites.
For customer Docker installation and boot startup use
[the Docker guide](docker-autostart.md) and [customer manual](customer_manual.md).
The current Docker service is `moboterra-current-stack.service`.

## Configuration ownership

`src/mobotic_config/config/platform.yaml` owns the physical topology, CAN,
geometry, motion limits, operator mapping and scanner addresses. Component YAML
files in the same directory own the driver, supervisor, operator, kinematics,
odometry, battery, safety and diagnostics policies.

The native launcher loads the installed `mobotic_config` package without
component overrides. On the existing robot, symlink installation connects this
package to the source configuration. Verify the installed package before assuming
that a different native build reads source edits. The current Docker deployment
explicitly mounts the central configuration directory read-only.

`scripts/setup_ros.bash` loads Jazzy, the native `install` and
`install-controller-types` overlays, then `config/raspberry-pi/ros.env`.
The current Docker image uses one merged overlay selected by
`MOBOTERRA_ROS_OVERLAY`. The default domain is 42 and ROS logs are under
`runtime/ros/log`.

## Native hardware startup

Stop the Docker service before native hardware operation:

```bash
sudo systemctl stop moboterra-current-stack.service
```

Confirm the host CAN link is already UP at the active profile's bitrate. Use
neutral controls, with B2 released. From the project root:

```bash
bash scripts/start_MoboTerra --check start_rviz:=false
bash scripts/start_MoboTerra start_rviz:=false
```

The check command validates configuration and selected visualization dependencies
without CAN access or robot startup. The hardware launch retains the shared
lock, duplicate/orphan checks, receiver recovery and verified shutdown guard.
Drive enable still requires a new B2 press. Stop with Ctrl+C and wait for the
drive-disable result. Do not run `start_ros2_manual_only.sh` alongside this stack.

## RViz and odometry

Omit `start_rviz:=false` from a native launch only in a graphical desktop session.
The launcher enables description and odometry by default. Closing RViz alone
does not stop the hardware stack; Ctrl+C in the launch terminal stops the launch.

To monitor the running Docker stack through the existing native installation:

```bash
source scripts/setup_ros.bash
rviz2 -d config/raspberry-pi/live.rviz --ros-args \
  -r __ns:=/moboterra \
  -r /tf:=/moboterra/tf -r /tf_static:=/moboterra/tf_static
```

The fixed frame is `odom`. Wheel feedback feeds ROMB measured body velocity,
then wheel odometry publishes `odom` to `base_link`. Missing or stale complete
feedback can prevent odometry publication. Another estimator must not publish
the same transform concurrently. Description is already owned by the integrated
`robot_state_publisher`.

## Recording and inspection

Use the existing native installation in another terminal:

```bash
source scripts/setup_ros.bash
bash scripts/record_steering_bag.sh --check
bash scripts/record_steering_bag.sh
```

New bags are saved under `runtime/bags/`. Ctrl+C stops the recorder. Keep ROS
domain, message definitions and namespace consistent with the running stack.

## Historical configuration migration

`config/managed/`, `config/systemd/`, `scripts/install_project_settings.py` and
`scripts/remove_managed_service.sh` maintain the older project-linked managed
deployment. `config/startup-policy.json` disables registration of that older
`moboterra-supervisor.service`; it does not disable the new current-stack service.

Those migration scripts are not prerequisites for current Docker installation.
The new service reads this checkout and its generated `runtime/docker-stack.env`,
rather than `/etc/moboterra`. Preserve old migration backups for service records.
