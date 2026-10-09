# MoboTerra

ROS 2 workspace for manual joystick driving and a supervised interface to external
autonomy. Autonomous planning/navigation is outside this project's scope.

The implementation is present in `src/`; it is not yet hardware-commissioned.
ROS integration and C++ build/test validation remain required before operation.
MoboTerra has four powered steering/traction modules (`front_left`, `front_right`,
`rear_left`, `rear_right`), eight actuator joints and no casters. User-confirmed
spacing follows actual CAD axes: 1.69 m front–rear / 0.99 m left–right, with the
old four-wheel radius 0.325 m and width 0.31 m. CAD Z=0 is confirmed as the floor.
CAN bindings and reference mounting offsets still require hardware checks.

## Control architecture

- Manual driving: `joy -> mobotic_manual_control -> mobotic_supervisor -> mobotic_kinematics -> mobotic_driver`.
- Autonomous velocity: `autonomy/cmd_vel -> supervisor -> cmd_vel -> kinematics -> kinematics/joint_setpoints -> driver`.
- Autonomy with its own kinematics: `autonomy/joint_setpoints -> supervisor -> supervisor/joint_setpoints -> driver`.
  `AUTO_DIRECT` is selectable as mode 2 without an extra permission setting.
- `wheel_modules/command` manages enable/disable and error clearing for **all units together**.
  It never carries joint motion. Hardware brake actuation follows enable; brake feedback is unavailable.
- The driver publishes `joint_states` and `wheel_modules/status`. Missing required CAN feedback
  blocks motion on all units; a new motion command is required after recovery.

All topic names above are relative to the robot namespace, normally `/moboterra`.
The supervisor consumes both `safety/state` and `safety/io_state`, plus
`battery/system_state`. One public `battery/state` supplies standard battery telemetry.

## Packages

| Package | Responsibility |
|:--|:--|
| [mobotic_config](src/mobotic_config/README.md) | Validated shared platform data, node policies and launch adapters |
| [mobotic_interfaces](src/mobotic_interfaces/README.md) | Shared messages and mode service |
| [mobotic_driver](src/mobotic_driver/README.md) | CAN drive control, joint feedback, management lease and drive diagnostics |
| [mobotic_kinematics](src/mobotic_kinematics/README.md) | Inverse kinematics and measured body velocity; not odometry |
| [mobotic_odometry](src/mobotic_odometry/README.md) | Measured planar wheel odometry, covariance and optional odom-to-base TF |
| [mobotic_manual_control](src/mobotic_manual_control/README.md) | Joystick-to-velocity teleop and deadman/source state |
| [mobotic_safety](src/mobotic_safety/README.md) | FlexiSoft TCP monitoring and conservative ROS interlocks |
| [mobotic_vanguard_battery](src/mobotic_vanguard_battery/README.md) | Read-only J1939 telemetry and aggregate readiness |
| [mobotic_supervisor](src/mobotic_supervisor/README.md) | Source arbitration, readiness gates and verified mode transitions |
| [mobotic_bringup](src/mobotic_bringup/README.md) | Integrated launch, scanners and platform diagnostics |
| [mobotic_description](src/mobotic_description/README.md) | CAD chassis/steering units, four CAD-aligned wheel modules and standalone robot_state_publisher |

## Configuration

All runtime YAML files live in [mobotic_config/config](src/mobotic_config/config/).
Edit `platform.yaml` for physical hardware, joint/CAN bindings, geometry, frames,
network endpoints and shared limits. The smaller YAML files contain only node
policies (timeouts, control mapping, odometry uncertainty, scanner acquisition).
Bringup, standalone launches, virtual actuators and description use the same loader.
See [configuration ownership, overrides and validation](src/mobotic_config/README.md).
No legacy YAML copies are generated. Existing node ROS parameter APIs are unchanged.

## Automated first-time installation

On Ubuntu 24.04 ARM64, download/unpack the project, open a terminal in its
directory and run:

```bash
bash ./install.sh
```

The command asks for Ubuntu administrator authentication, installs missing host
packages and Docker/Compose, and builds the runtime, GUI and offline CAD tools.
ROS dependencies are installed inside Docker; no host ROS installation or manual
pip commands are needed. The build runs the full ROS tests, validates the supplied
configuration and starts an isolated virtual stack to check all eight joints,
odometry, wheel status and diagnostics. The installer also checks the installed
entrypoint, healthcheck, RViz, ROS Graph and CAD imports. It installs `moboterra-gui`,
`moboterra-tools` and desktop application entries. The default prepares software
without CAN adapters and does not configure or start a robot service.

After installation:

```bash
moboterra-gui rviz2
moboterra-gui rqt_graph
moboterra-tools  # writes moboterra-preview.png in the current directory
```

When the two adapters are available, run `sudo bash ./deploy/install.sh --no-start`
to identify their real serials and install the hardware service. Complete the
hardware commissioning checks before starting it. Scanner Ethernet addresses and
physical safety wiring remain specific to the connected hardware.
See [deployment instructions](deploy/README.md) for unattended provisioning,
direct Docker builds, recovery and hardware startup.

## Build and launch

On a ROS 2 Linux build host (target: Jazzy), from the MoboTerra repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
ros2 run mobotic_config check_config
ros2 launch mobotic_bringup moboterra.launch.py \
  stack_type:=hardware robot_name:=moboterra \
  can_interface:=can0 battery_can_interface:=can1
```

Install the external `ros2_socketcan`, `joy` and `sick_safetyscanners2` packages
and their dependencies. Bringup launches the current stack automatically; it does
not configure the OS CAN interface, NIC address or hardware safety wiring. Scanner
TF is supplied by the separately launched `mobotic_description`, not by the scanners.
The drive bridge uses `can0`; a separate receive-only battery receiver uses
`can1` and publishes `battery_can/can_rx`. Configure both OS interfaces at
500 kbit/s, with the battery adapter in listen-only mode, as in the reference.
No battery operating-consent/HV control is added.

Bringup now also starts measured wheel odometry:
`joint_states -> kinematics -> agv_vel -> mobotic_odometry`.
It publishes `/moboterra/odometry` and `odom -> base_link` on `/moboterra/tf`.
The default `publish_odom_tf:=auto` honors the shared odometry policy.
Use `publish_odom_tf:=false` if an external estimator owns that transform, or
`start_odometry:=false` to omit the node. Robot-state publisher remains a separate
description launch. No laser/IMU fusion or autonomous navigation is added.

The four-wheel profile restores the 10 ms heartbeat and 116053 increments/s
traction ceiling. Driver acceleration/deceleration use the actual 0.325 m wheel
radius (32093 increments/s^2 for nominal 1 m/s^2); the supervisor command ramp
remains 0.5 m/s^2. See the [bringup configuration](src/mobotic_bringup/README.md).

Before launching on a secured platform, verify the configuration against the
machine. Scanner defaults: front `10.60.20.52`, rear `10.60.20.53`,
PC NIC `10.60.20.186` (override with `scanner_host_ip`).
The supervisor automatically requests enable when safety and battery are ready
and minimum SOC is at least 20%, even without an active joystick command.
The default USB profile requires no held button for manual motion: left stick
steers, right stick translates/crabs, and L1 or R1 boosts. Manual motion still
requires fresh joystick input and all supervisor readiness checks.

## Readiness and loss handling

| Condition | Software response |
|:--|:--|
| Safety/battery missing, stale or not ready; SOC below 20% | Supervisor requests stop + disable |
| Joint-command loss (driver default 0.1 s) | Zero motion |
| Supervisor management loss (0.3 s, steady-clock check) | Stop + disable, cancel enable retries |
| Required CAN motion/status loss (0.3 s) or fault-data loss (2.5 s) | Stop all units; inhibit motion and standstill verification |
| Unrepresentable steering position | Reject whole joint command before CAN writes; invalid selected autonomy candidate clears its cache and requests stop |
| Mode change | Stop, confirm fresh sustained standstill, then accept only new commands |
| Measured velocity loss | Odometry/odom TF publication stops; recovery rebases without guessing missing travel and increases uncertainty |

ROS monitoring/control is not safety-rated. ELMO/FlexiSoft and the independent
hardware stop/brake chain remain responsible for physical safety.

## Documentation and remaining work

- [Whole-project node/topic graph](docs/moboterra_rqt_graph.md): source-derived hardware and virtual layouts.
- [Supervisor interface contract](docs/supervisor_interfaces.md): topics, modes and gates.
- [Hardware monitoring and commissioning](docs/hardware_monitoring.md): battery/scanner mapping, STO limitations, diagnostics and loss tests.

Still required: ROS/C++ build and integration tests, secured hardware commissioning,
virtual-stack ROS runtime validation, an explicit fault-reset policy and joystick
fault-reset controls and the final REMdevice CANopen input driver/mapping.
The temporary F310/F710 profile uses Back to toggle MANUAL/AUTO_VELOCITY and
uses no face buttons. AUTO_DIRECT is terminal/service-only. Joystick and
terminal requests share stop/standstill/switch logic; the driver enforces one
selected motion path. The [robot description](src/mobotic_description/README.md)
now includes the updated CAD geometry and a standalone `robot_state_publisher`
launch; CAD floor/axis placement is confirmed, but ROS/RViz and hardware validation remain required.
It is not automatically included in bringup. Wheel odometry and `odom -> base_link`
are implemented, but need ROS/hardware validation and covariance/slip calibration;
Nominal CAD-aligned scanner frames are included in the description; physical
extrinsic calibration remains required. `map -> odom` localization is not added.
Unknown STO bit mappings are exposed as raw telemetry, not decoded safety confirmation.

## Container deployment

An ARM64 Ubuntu 24.04 deployment is available in [deploy/README.md](deploy/README.md).
It builds a ROS 2 Jazzy runtime image, identifies the two USB-to-CAN adapters by
their unique USB serial numbers, configures platform `can0` and listen-only
battery `can1`, and starts the hardware stack under a single systemd supervisor.
The supervisor stops ROS on adapter loss and performs a clean, health-checked
restart after both adapters return. See the deployment guide for installation,
rollback, diagnostics, GUI tools, and hardware acceptance tests.
