# MoboTerra command reference

Run these commands on the Ubuntu robot computer. ROS and RViz execute inside the
installed Docker image; no native ROS installation or host ROMB runtime is needed.
Run installation and updates from the supplied project root. After installation,
`moboterra-ctl` can be used from any directory while that project remains in place.

## Installation and updates

```bash
bash ./install.sh
```

This obtains administrator access, installs missing Docker/host utilities,
builds and checks the complete image, installs `moboterra-ctl` and enables boot
startup. The stack remains stopped until started explicitly or the next boot.
`bash deploy/install.sh` is equivalent. The image contains the original ROMB
package, its guest libraries and target emulator, ROS 2 Jazzy and RViz.

| Option | Result |
| --- | --- |
| `--start` | Enable boot startup and start ROS now; B2 still controls drive enable |
| `--no-start` | Install with boot startup disabled; leave stopped |
| `--software-only` | Build/check image and install CLI; do not register a robot service |
| `--image IMAGE` | Use a prepared complete image instead of building source |
| `--ros-domain-id ID` | Set the project ROS domain, 0–232 |

On an existing current installation, `--software-only` preserves the selected
deployment image and ROS domain while building/checking another image.

For updates, stop first, preserve local configuration/work, review the intended
branch and run the installer again. Pull only when local work is safely handled:

```bash
sudo moboterra-ctl stop
git status --short
git branch --show-current
git pull --ff-only
bash ./install.sh
```

## Host service commands

| Command | Result |
| --- | --- |
| `sudo moboterra-ctl start` | Start ROS; wait for normal B2 and readiness conditions |
| `sudo moboterra-ctl stop` | Request orderly shutdown and drive-disable verification |
| `sudo moboterra-ctl restart` | Stop and start the stack deliberately |
| `sudo moboterra-ctl status` | Show service, container health and image identity |
| `sudo moboterra-ctl enable` | Enable boot startup without starting now |
| `sudo moboterra-ctl enable --now` | Enable boot startup and start now |
| `sudo moboterra-ctl disable --now` | Stop now and disable future boot startup |

The managed service is `moboterra-current-stack.service`; the robot container
is `moboterra-current-stack`. B2 initializes/enables and is not a stop toggle.
Steering can move during alignment even while traction is zero.

## Logs and CAN

```bash
sudo moboterra-ctl logs --lines 200
sudo moboterra-ctl logs --follow
sudo moboterra-ctl logs --host --lines 100
sudo moboterra-ctl logs --host --follow
sudo moboterra-ctl can
```

Default logs are ROS container output; `--host` selects the service journal.
Ctrl+C exits the log display without stopping ROS. `can` inspects the configured
host CAN link without transmitting. The active profile uses shared transmitting
`can0` at 500000 bit/s, including batteries; do not apply the old `can1` setup.

## Enter the current ROS environment

Prefix ordinary ROS commands with `sudo moboterra-ctl ros`:

```bash
sudo moboterra-ctl ros node list
sudo moboterra-ctl ros topic list -t
sudo moboterra-ctl ros service list -t
sudo moboterra-ctl ros topic echo --once /moboterra/vehicle/mode
sudo moboterra-ctl ros topic echo --once /moboterra/operator/state
sudo moboterra-ctl ros topic echo --once /moboterra/wheel_modules/status
sudo moboterra-ctl ros topic echo --once /moboterra/battery/system_state
sudo moboterra-ctl ros topic echo --once /moboterra/vehicle/odom
sudo moboterra-ctl ros interface show mobotic_interfaces/msg/VehicleModeState
```

These require the current container to be running. Some topic commands wait for
a publisher; Ctrl+C leaves them. For a sourced interactive container shell:

```bash
sudo moboterra-ctl shell
```

Inside that shell, run `ros2 ...` directly. Plain `docker exec ... bash` does not
source the project's domain and log settings.

## Drive enable and fault recovery

Normal operation uses B2 with neutral controls. The equivalent enable request is:

```bash
sudo moboterra-ctl ros service call /moboterra/wheel_modules/enable_drives \
  std_srvs/srv/Trigger '{}'
```

The separate fault-clear transaction is:

```bash
sudo moboterra-ctl ros service call /moboterra/wheel_modules/clear_errors \
  std_srvs/srv/Trigger '{}'
```

Correct the cause first. Acceptance is not completed recovery. Observe
`vehicle/mode`, including reset progress, fault reasons and readiness. The reset
deadline is 20 seconds; the driver startup attempt is bounded to 15 seconds.
Missing motion data alone is not proof of standstill.

## Select an operating mode

Request only the intended value, for example MANUAL:

```bash
sudo moboterra-ctl ros service call /moboterra/vehicle/set_mode \
  mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 0}'
```

Selectable values are 0 MANUAL, 1 AUTONOMOUS body velocity and 2 KINEMATICOFF
steering position/traction current. Values 3, 4 and 255 are automatic states.
See [the interface reference](ros_interface_manual.md) before integrating modes 1 or 2.
The separate standstill-guarded wheel-odometry reset is:

```bash
sudo moboterra-ctl ros service call /moboterra/vehicle/reset_odometry \
  std_srvs/srv/Trigger '{}'
```

## Configuration check and rollback

```bash
sudo moboterra-ctl check
sudo moboterra-ctl rollback
```

`check` validates the current configuration and selected dependencies in an
isolated container without physical networking, CAN or robot startup. It does
not use the host's runtime lock or require a native ROS environment.

`rollback` selects the previous installed image after stopping ROS and checking
the current configuration against it. It retains current configuration and leaves
ROS stopped. If validation fails, the image selection is unchanged. An explicit
`rollback --start` also starts ROS afterward; ordinary B2/readiness checks remain.

## Native maintenance and RViz

For normal customer visualization, use the bundled RViz from a logged-in robot
desktop with X11/XWayland:

```bash
moboterra-ctl rviz
```

The command uses the image's ROS, current project display file, domain, namespace
and TF remappings. It requests Docker sudo access if required. No native ROS or
`xhost` access-policy change is needed. Closing RViz does not stop the robot.
Existing native launchers remain engineering tools; they require their native
overlays/runtime and must never run alongside the Docker hardware stack.

See [the customer manual](customer_manual.md) and [deployment guide](docker-autostart.md).
