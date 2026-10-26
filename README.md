# MoboTerra

ROS 2 workspace for manual joystick driving and a supervised interface to external
autonomy. Autonomous planning/navigation is outside this project's scope.

The implementation is present in `src/`; it is not yet hardware-commissioned.
ROS integration and C++ build/test validation remain required before operation.
The [software readiness review](docs/software_readiness.md) records the current
checks, corrected startup dependencies and remaining target-machine validation.
MoboTerra defaults to four powered steering/traction modules (`front_left`, `front_right`,
`rear_left`, `rear_right`), eight actuator joints and no casters. User-confirmed
spacing follows actual CAD axes: 1.69 m front–rear / 0.99 m left–right, with the
old four-wheel radius 0.325 m and width 0.31 m. CAD Z=0 is confirmed as the floor.
CAN bindings and reference mounting offsets still require hardware checks.
The same application also supports two powered modules: select the required
records in the platform config's `modules` list. Driver bindings, feedback gates,
kinematics, odometry, virtual drives and the robot model follow that list.
See the [configuration and two-module setup](docs/two_module_setup.md); the
front-right/rear-left profile uses a 250-kbit/s platform bus, PAIL receiver node
10, master node 11 and the confirmed B1/SP3 and B2/SP4 control mapping.

The default `/moboterra/operator` provider uses PAIL/Ecobox CANopen node 9 on
`can0` at 500 kbit/s, without a hold-to-drive button. Physical control mapping
is still required; the unverified profile publishes zero motion and permits
passive capture. See the [PAIL capture guide](docs/remdevice_commissioning.md).

## Control architecture

- Operator/autonomous velocity: candidate -> supervisor -> `vehicle/cmd_vel` -> kinematics -> `kinematics/joint_setpoints` -> supervisor -> `wheel_modules/command` -> driver.
- Autonomy with its own kinematics: `autonomy/joint_setpoints` -> supervisor -> the same final `wheel_modules/command` array -> driver.
  KINEMATICOFF is mode 2; velocity candidates have no motion effect there.
- The supervisor contains the mode state machine and publishes `vehicle/mode`, including automatic SAFETY and latched ERROR. Operator inputs are separately published on `operator/state`.
- Internal `wheel_modules/operation_command` manages enable/disable and explicit reset transactions for all units together. Public reset is supervisor-owned `wheel_modules/clear_errors`.
- The driver publishes `joint_states` and `wheel_modules/status`. Missing required CAN feedback
  blocks motion on all units; a new motion command is required after recovery.

Physical startup waits for supervisor readiness, then initializes controller modes/PDOs/ramps and enables drives with zero targets, following the working EC flow. Motion feedback is acquired after enable and is required before driving. B2 requests verified fault reset and re-enable. Encoder reference and saved parameters are retained. See [drive startup and B2 behavior](docs/drive_startup.md). Physical operation still requires hardware validation.

All topic names above are relative to the robot namespace, normally `/moboterra`.
FlexiSoft profiles require both `safety/state` and `safety/io_state`. The explicit
two-module hardware-STO profile requires no Siemens network telemetry. Both
variants require `battery/system_state`. One public `battery/state` supplies standard battery telemetry.

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
configuration and starts isolated four- and two-module virtual stacks to check
enabled drives, supervisor readiness, joints, odometry and diagnostics. The installer also checks the installed
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
See the [installation and command reference](docs/commands.md) for copyable
host/container commands, isolated virtual testing, updated operator/vehicle ROS
services and the edit–rebuild–deploy workflow. Rebuild and verify the image's
message definitions after interface changes.

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
TF is supplied by integrated `robot_state_publisher` (`start_description:=true` by default), not by the scanners. Do not start a second description owner.
The drive bridge uses `can0`; a separate receive-only battery receiver uses
`can1` and publishes `battery_can/can_rx`. Use each bus's bitrate from the selected
platform config, with the battery adapter in listen-only mode. The managed host
applies those rates; a direct ROS launch does not configure host interfaces.
No battery operating-consent/HV control is added.

Bringup now also starts measured wheel odometry:
`joint_states -> kinematics -> vehicle/measured_velocity -> mobotic_odometry`.
It publishes `/moboterra/vehicle/odom` and `odom -> base_link` on `/moboterra/tf`.
The default `publish_odom_tf:=auto` honors the shared odometry policy.
Use `publish_odom_tf:=false` if an external estimator owns that transform, or
`start_odometry:=false` to omit the node. Use `start_description:=false` only when another description owner is already running. No laser/IMU fusion or autonomous navigation is added.

The four-wheel profile restores the 10 ms heartbeat and 116053 increments/s
traction ceiling. The virtual driver acceleration/deceleration use the actual 0.325 m wheel
radius (32093 increments/s^2 for nominal 1 m/s^2); physical startup applies the selected ramps. The supervisor command ramp
remains 0.5 m/s^2. See the [bringup configuration](src/mobotic_bringup/README.md).

Before launching on a secured platform, verify the configuration against the
machine. Scanner defaults: front `10.60.20.52`, rear `10.60.20.53`,
PC NIC `10.60.20.186` (override with `scanner_host_ip`).
The supervisor requests initialization/enable when battery, applicable safety,
configured operator and required host checks pass, with minimum SOC at least 20%.
An active motion command is not required to initialize or enable.
The default USB profile requires no held button for manual motion: left stick
steers, right stick translates/crabs, and L1 or R1 boosts. Manual motion still
requires fresh joystick input and all supervisor readiness checks.
Startup, reconnect, mode changes and safety recovery additionally require neutral controls; a configured deadman requires release/new press. USB E-stop state is unknown rather than fabricated.

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
- [Wheel command path and latency](docs/control_path_and_latency.md): EC comparison, exact final-message fields, configured timing and measurement limits.
- [ROS interface manual](docs/ros_interface_manual.md): topic/message descriptions, modes and recovery.
- [Supervisor interface contract](docs/supervisor_interfaces.md): topics, modes and gates.
- [Hardware monitoring and commissioning](docs/hardware_monitoring.md): battery/scanner mapping, STO limitations, diagnostics and loss tests.

Still required: ROS/C++ build and integration tests, secured hardware commissioning,
virtual-stack ROS runtime validation, an explicit fault-reset policy and joystick
fault-reset hardware acceptance and the physical PAIL control mapping. The
REMdevice CAN provider and B2 supervisor-reset path are implemented; the default
unverified profile permits capture only.
The temporary F310/F710 profile uses Back to toggle MANUAL/AUTO_VELOCITY and
uses no face buttons. AUTO_DIRECT is terminal/service-only. Joystick and
terminal requests share stop/standstill/switch logic; the driver enforces one
selected motion path. The [robot description](src/mobotic_description/README.md)
now includes the updated CAD geometry and a standalone `robot_state_publisher`
launch; CAD floor/axis placement is confirmed, but ROS/RViz and hardware validation remain required.
It is included in bringup by default. Wheel odometry and `odom -> base_link`
are implemented, but need ROS/hardware validation and covariance/slip calibration;
Nominal CAD-aligned scanner frames are included in the description; physical
extrinsic calibration remains required. `map -> odom` localization is not added.
Unknown STO bit mappings are exposed as raw telemetry, not decoded safety confirmation.

## Container deployment

An ARM64/AMD64 Ubuntu 24.04 deployment is available in [deploy/README.md](deploy/README.md).
It builds a ROS 2 Jazzy runtime image, identifies the two USB-to-CAN adapters by
their unique USB serial numbers, configures platform `can0` and listen-only
battery `can1`, and starts the hardware stack under a single systemd supervisor.
Adapter loss and host-health faults keep ROS alive and latch ERROR with motion
inhibited. Recovery requires explicit reset rather than a whole-stack restart.
See the deployment guide for installation,
rollback, diagnostics, GUI tools, and hardware acceptance tests.
