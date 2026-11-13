# Pi settings and startup

The actual MoboTerra hardware configuration and component tuning are owned by `src/mobotic_config/config/`. Native startup, the manual-only wrapper and joystick recovery load this package directly.

Edit `platform.yaml` for hardware, geometry, CAN IDs, motion limits, joystick mapping and scanner addresses. Edit the component YAML files for polling periods, homing, source scaling, timeouts and other node policies. Native startup does not load component override files. The installed package in this workspace uses symlinks to these source files, so changes apply on the next stack start without rebuilding.

| Location relative to MoboTerra-main | Contents |
|---|---|
| `src/mobotic_config/config/platform.yaml` | Actual hardware, geometry, CAN, joystick, scanner, battery and safety configuration |
| `src/mobotic_config/config/driver.yaml` | Actual feedback polling, heartbeat, steering alignment and driver deadlines |
| `src/mobotic_config/config/supervisor.yaml` | Actual supervisor control period, readiness and recovery policies |
| `src/mobotic_config/config/manual_control.yaml` | Actual operator publication rate, scaling and receiver transmission policy |
| `src/mobotic_config/config/` | Kinematics, odometry, diagnostics, scanner, battery and safety node policies |
| `config/raspberry-pi/platform-can0-500k-micontrol.yaml` | Compatibility symlink to the central `platform.yaml` |
| `config/raspberry-pi/bag-qos.yaml` | Recording QoS overrides |
| `config/raspberry-pi/ros.env` | ROS domain and project-local ROS log locations |
| `config/raspberry-pi/` | Existing virtual profiles, RViz/DDS settings and auxiliary configuration |
| `config/managed/` | Existing managed deployment environment, platform, overrides and Compose configuration |
| `config/managed/installed-defaults/` | Exact configuration defaults captured from the installed host release |
| `config/systemd/` | Pi service and tmpfiles configuration |
| `scripts/` | Native startup, environment, recording and settings installation entry points |
| `tools/pi/` | CAN recording/export and guarded receiver-recovery utilities |
| `scripts/legacy/`, `tools/legacy/` | Preserved older recovery utilities |
| `runtime/` | Generated logs, new recordings, lock files, migration status and originals preserved during migration; ignored by Git |

The native launcher uses `Configuration()` from the installed `mobotic_config` package with no component overrides. Centralization preserved the previous native hardware profile and all nine effective component parameter sets exactly; the pre-migration values are covered by the configuration regression baseline. Historical managed-container and virtual launch profiles remain separate from native physical startup.

The feedback timing policy in `src/mobotic_config/config/` adds 50 ms of margin: `driver.yaml` motion/status, `kinematics.yaml` joint feedback, `odometry.yaml` feedback, and `supervisor.yaml` wheel/velocity feedback use 0.35 s. In `diagnostics.yaml`, `odometry_timeout` and the derived supervisor `health_input_timeout` use 0.55 s. These values apply on the next stack start. Joystick/scanner deadlines, command watchdog, supervisor lease, and health-report freshness retain their existing values.

The old `~/.config/moboterra`, `~/moboterra_tools`, home-directory launch scripts and steering playback RViz filename are compatibility symlinks. They do not contain independent settings. The interactive shell sources `scripts/setup_ros.bash`; domain ID and workspace setup are no longer maintained as independent shell settings.

## Boot policy: manual startup

`config/startup-policy.json` sets `managed_systemd_service` to `false`. The settings installer respects this choice and will not register the MoboTerra system service again. Docker's own service is independent and remains available.

Remove the existing robot service registration once with:

```bash
sudo /home/mobotic/Downloads/MoboTerra-main/scripts/remove_managed_service.sh
```

This stops/disables the robot supervisor, removes its systemd link and reloads systemd. The service definition remains in `config/systemd/`; project settings and Docker are retained. Check completion with `systemctl status moboterra-supervisor.service`, which should report that the unit could not be found.

## System configuration links

Root-owned configuration requires one administrative installation step. Keep both stacks stopped, then run:

```bash
sudo python3 /home/mobotic/Downloads/MoboTerra-main/scripts/install_project_settings.py
```

The installer rejects changed original files, preserves originals and historical backups under `runtime/system-settings-before-*/`, replaces the system configuration paths with symlinks, validates the unit, and reloads systemd. It does not start, restart or enable the service, access CAN, or change robot policies. If installation fails, it restores the system paths changed by that attempt.

The links are:

- `/etc/moboterra` → `config/managed/`
- `/opt/moboterra/current/config` → `config/managed/installed-defaults/`
- `/opt/moboterra/current/compose.yaml` → `config/managed/compose.yaml`
- The service unit link is omitted while `managed_systemd_service` is false.
- `/etc/tmpfiles.d/moboterra.conf` → `config/systemd/moboterra-tmpfiles.conf`

The staged service explicitly selects project-owned managed configuration. `ProtectHome=read-only` lets the service read that configuration under `/home` while keeping it read-only inside the service. Existing installed executables, ROS packages and Docker images remain installed dependencies.

Check pending or completed system links without sudo:

```bash
python3 /home/mobotic/Downloads/MoboTerra-main/scripts/install_project_settings.py --check
```

After a completed migration, systemd configuration edits still need `sudo systemctl daemon-reload`. Configuration changes take effect on the next appropriate stack start; this installer leaves the managed service stopped. Keep the checkout at this path while system links use it. To relocate it, update the integration paths deliberately.

## Manual start

If the managed service is still registered and running, stop it first:

```bash
sudo systemctl stop moboterra-supervisor.service
```

The physical CAN interface must already be configured, and the existing hardware stop and scanner requirements still apply. Start with the joystick neutral and B2 released:

```bash
/home/mobotic/Downloads/MoboTerra-main/scripts/start_ros2_manual_only.sh
```

Keep the launch terminal open. Stop using Ctrl+C and wait for verified drive-disable shutdown. The launcher retains its duplicate-stack, receiver recovery, CAN and shutdown checks.

## RViz, robot TF and wheel odometry

Both `scripts/start_ros2_four_micontrol.sh` and the manual-only wrapper now enable robot description, wheel odometry and RViz by default. RViz is part of the same ROS launch, so Ctrl+C in the launch terminal also closes the viewer. Closing just the RViz window leaves the robot stack running.

The live display settings are in `config/raspberry-pi/live.rviz`. They show the robot model, TF tree, wheel-odometry history and both scanner point streams. The fixed frame is `odom`; the viewer follows `base_link`. Topics are relative to the `/moboterra` namespace, including `robot_description`, `vehicle/odom`, `tf` and `tf_static`.

Odometry integrates `vehicle/measured_velocity` from measured wheel feedback through the existing ROMB kinematics adapter. It does not integrate joystick commands. The robot description publishes the body/scanner and articulated wheel transforms; odometry publishes `odom` → `base_link`. A fresh complete wheel sample is required before odometry starts. Until fresh feedback is available after enable/recovery, RViz may report that the `odom` frame is missing. Stale feedback stops new odometry/TF updates, as before.

For a headless terminal or to save graphics resources, retain TF and odometry but omit the viewer:

```bash
/home/mobotic/Downloads/MoboTerra-main/scripts/start_ros2_four_micontrol.sh start_rviz:=false
```

The manual-only wrapper accepts the same option. Individual `start_description:=false`, `start_odometry:=false`, `publish_odom_tf:=false` and `rviz_config:=/path/to/view.rviz` launch overrides remain available. Use `publish_odom_tf:=false` if another estimator owns the odometry transform; choose a matching RViz fixed frame when disabling an expected transform.

Startup validates the selected executables, rendered robot model, mesh resources and RViz settings before receiver recovery or CAN writers are started. A graphical startup requires a desktop display; `--check` validates configuration without opening a window or communicating with the robot.

## Record in another terminal

```bash
/home/mobotic/Downloads/MoboTerra-main/scripts/record_steering_bag.sh
```

This loads Jazzy, both workspace overlays and custom message types. New bags go to `runtime/bags/steering_<timestamp>` inside this checkout. Existing recordings in Downloads are unchanged. Stop the recorder with Ctrl+C in its terminal.

For an interactive ROS terminal:

```bash
source /home/mobotic/Downloads/MoboTerra-main/scripts/setup_ros.bash
```

Validation-only commands perform no robot startup or CAN communication:

```bash
/home/mobotic/Downloads/MoboTerra-main/scripts/start_ros2_four_micontrol.sh --check
/home/mobotic/Downloads/MoboTerra-main/scripts/start_ros2_manual_only.sh --check
/home/mobotic/Downloads/MoboTerra-main/scripts/record_steering_bag.sh --check
```

Active settings, launchers and integration definitions are visible to Git. Generated data, historical backups and migration originals are ignored. Moving configuration does not create a Git commit; include the new project files in the normal review and backup process.
