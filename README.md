# MoboTerra

ROS 2 workspace for manual joystick driving and a supervised interface to external
autonomy. Autonomous planning/navigation is outside this project's scope.

The implementation is present in `src/`; it is not yet hardware-commissioned.
Offline Python checks and the standalone C++ motion-authority checks have passed.
Full ROS 2 Jazzy build/integration and physical acceptance remain required before operation.
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

Select the platform profile that matches the machine before hardware launch:

| Profile | Modules | Platform CAN | PAIL operator |
|:--|:--|:--|:--|
| [platform.yaml](src/mobotic_config/config/platform.yaml), default | Four steering/traction modules | `can0`, 500 kbit/s | Receiver node 9; mapping unverified, zero motion/capture only |
| [platform-front-right-rear-left.yaml](src/mobotic_config/config/platform-front-right-rear-left.yaml) | Front-right nodes 3/4 and rear-left nodes 7/8 | `can0`, 250 kbit/s | Receiver node 10, master node 11; confirmed stick, RP1, B1 and B2 assignments |

Both profiles use a separate battery bus. The two-module PAIL profile uses RP1
for Manual/Autonomous selection, B1/SP3 for boost and B2/SP4 for reset, with no
hold-to-drive button. RP2 Safety Enable/Disable has no confirmed ROS input mapping;
its physical safety function must be verified independently. See the
[PAIL capture and commissioning guide](docs/remdevice_commissioning.md).

## Control architecture

- Operator/autonomous velocity: candidate -> supervisor -> `vehicle/cmd_vel` -> kinematics -> `kinematics/joint_setpoints` -> supervisor -> `wheel_modules/command` -> driver.
- Autonomy with its own kinematics: `autonomy/joint_setpoints` -> supervisor -> the same final `wheel_modules/command` array -> driver.
  KINEMATICOFF is mode 2; velocity candidates have no motion effect there.
- The supervisor contains the mode state machine and publishes `vehicle/mode`, including automatic SAFETY and latched ERROR. Operator inputs are separately published on `operator/state`.
- Internal `wheel_modules/operation_command` manages enable/disable and explicit reset transactions for all units together. Public reset is supervisor-owned `wheel_modules/clear_errors`.
- The driver publishes `joint_states` and `wheel_modules/status`. Missing required CAN feedback
  blocks motion on all units; a new motion command is required after recovery.

Physical startup follows the EC order: launch, initialize, then enable. The
supervisor and driver perform this sequence automatically; there is no separate
manual physical `initialize_all_drives` / `enable_all_drives` step. See the
[startup sequence below](#physical-startup-and-recovery).

All topic names above are relative to the robot namespace, normally `/moboterra`.
FlexiSoft profiles require both `safety/state` and `safety/io_state`. The explicit
two-module hardware-STO profile requires no Siemens network telemetry. Both
variants require `battery/system_state`. Both platform profiles require two
Vanguard fixed packs directly on the shared, listen-only 500-kbit/s battery bus.
`battery/battery1/state` and `battery/battery2/state` expose their individual
measurements. `battery/state` aliases pack 1; the supervisor uses aggregate
readiness and the **lower SOC**, requiring both packs ready and at least 20% SOC.
The retained reference source addresses/PGNs need verification for the installed
packs; see [battery telemetry and commissioning](src/mobotic_vanguard_battery/README.md).

## Packages

| Package | Responsibility |
|:--|:--|
| [mobotic_config](src/mobotic_config/README.md) | Validated shared platform data, node policies and launch adapters |
| [mobotic_interfaces](src/mobotic_interfaces/README.md) | Shared messages and mode service |
| [mobotic_driver](src/mobotic_driver/README.md) | CAN drive control, joint feedback, management lease and drive diagnostics |
| [mobotic_kinematics](src/mobotic_kinematics/README.md) | Inverse kinematics and measured body velocity; not odometry |
| [mobotic_odometry](src/mobotic_odometry/README.md) | Measured planar wheel odometry, covariance and optional odom-to-base TF |
| [mobotic_manual_control](src/mobotic_manual_control/README.md) | PAIL CAN/USB operator input, velocity candidates, mode/reset requests and source state |
| [mobotic_safety](src/mobotic_safety/README.md) | Direct SICK field monitoring, FlexiSoft TCP monitoring and ROS interlocks |
| [mobotic_vanguard_battery](src/mobotic_vanguard_battery/README.md) | Read-only J1939 telemetry and aggregate readiness |
| [mobotic_supervisor](src/mobotic_supervisor/README.md) | Source arbitration, scanner speed limits, readiness, startup/recovery and verified mode transitions |
| [mobotic_bringup](src/mobotic_bringup/README.md) | Integrated launch, scanners and platform diagnostics |
| [mobotic_description](src/mobotic_description/README.md) | CAD chassis/steering units, configured wheel modules and robot_state_publisher |

## Configuration

All runtime YAML files live in [mobotic_config/config](src/mobotic_config/config/).
Edit `platform.yaml` for physical hardware, joint/CAN bindings, geometry, frames,
network endpoints and shared limits. The smaller YAML files contain only node
policies (timeouts, control mapping, odometry uncertainty, scanner acquisition).
Bringup, standalone launches, virtual actuators and description use the same loader.
See [configuration ownership, overrides and validation](src/mobotic_config/README.md).
No legacy YAML copies are generated. The supervisor's `warning_speed_limit` is
an absolute speed in m/s (default `0.3`); it replaces the former proportional
`warning_speed_scale` policy.

## Scanner fields and speed supervision

The safety adapter reads each SICK microScan3's `RawMicroScanData` directly on
`scanner/front_left/raw_data` and `scanner/rear_right/raw_data`, then publishes
`scanner/<location>/safety_zone_status`. Both scanners must provide fresh, valid
field status, including in the two-module `hardwired_sto` profile.

| Scanner condition | Supervisor response |
|:--|:--|
| Yellow warning field occupied | Ramp down to a maximum of **0.3 m/s**; preserve any lower requested speed |
| Red protective field occupied | Immediately command **0 m/s** and inhibit motion |
| Both scanners clear | Remove the scanner speed cap; normal limits and readiness checks apply |
| Either scanner missing, stale or invalid | Stop/inhibit motion; loss after startup latches a scanner-monitor fault requiring explicit recovery |

Yellow uses the normal acceleration/deceleration ramp, so speed can remain above
0.3 m/s while slowing. The policy covers velocity commands and direct wheel
commands. Red clears cached motion; recovery requires neutral/rearm where
applicable and a new command. An immediate zero command does not establish a
physical stopping time.

Valid protective evaluation paths identify red; valid warning evaluation paths
identify yellow. These assignments must match the commissioned scanner
configuration. Physical OSSD and field-set state are not inferred from those
messages. ROS monitoring is not safety-rated; the independent scanner/safety
controller/STO chain remains responsible for physical safety.

## Physical startup and recovery

1. Launch the hardware stack with the selected platform config. Before enable
   permission, the driver reads controller state/faults and sends its heartbeat.
2. The supervisor waits for fresh scanner fields, battery readiness/HV and SOC
   of at least 20%, the configured operator interlock, applicable safety/STO
   conditions and host checks when required. No motion command is needed.
3. The driver disables/zeros targets, performs the initial fault-clear sequence
   once, configures PDOs, controller modes and traction limits/ramps, and verifies
   configuration acknowledgements/readback for every configured drive.
4. Only after configuration completes does it enable the drives with zero
   targets. Position/velocity feedback may become available only after enable.
5. Fresh, complete, enabled and fault-free drive feedback completes startup and
   permits motion. Missing feedback from any required module blocks all modules.

In the two-module profile, Elmo nodes **4 and 8** publish traction feedback through
TPDOs, including position through TPDO3. SYNC starts after all configured drives
(including steering nodes 3 and 7) finish configuration; joystick motion is not
needed to start feedback.

Startup has a 15 s deadline. Failed setup, timeout or lost permission requests
stop/disable; investigate a failed startup and restart the driver to retry.
Runtime B2 requests supervisor-owned verified fault reset and zero-target
re-enable, with neutral controls and ready interlocks. Encoder reference and
saved parameters are retained; startup does not home or zero an encoder.
See [drive startup and B2 recovery](docs/drive_startup.md) for admission checks,
fault-clearance acknowledgements and standstill verification.

## Automated first-time installation

On Ubuntu 24.04 ARM64 or AMD64, download/unpack the project, open a terminal in its
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

This launches the default four-module profile. For the two-module robot, add
`platform_config:=src/mobotic_config/config/platform-front-right-rear-left.yaml`.
On a deployed host, use the commissioned `/etc/moboterra/platform.yaml` instead.
Hardware launch can initialize and enable drives automatically once readiness
passes; secure the machine and verify the selected configuration first.

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
The default operator source is PAIL CAN. USB joystick support is an alternative
that requires its own verified configuration. Manual motion requires fresh
operator input and all supervisor readiness checks.
Startup, reconnect, mode changes and safety recovery additionally require neutral controls; a configured deadman requires release/new press. USB E-stop state is unknown rather than fabricated.

Check the deployed configuration and live readiness without requesting motion:

```bash
ros2 run mobotic_bringup check_readiness \
  --platform-config /etc/moboterra/platform.yaml
```

Add `--offline` for configuration validation without ROS or CAN access.

## Readiness and loss handling

| Condition | Software response |
|:--|:--|
| Safety/battery missing, stale or not ready; SOC below 20% | Supervisor requests stop + disable |
| Scanner warning/protective/unknown state | Apply the field policy above; both scanner streams remain required |
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

Still required: full ROS 2 Jazzy/C++ build and integration validation, virtual-stack
runtime validation on the target, and secured hardware commissioning. Physical
acceptance must cover scanner field assignments and stop/recovery, B2 fault
reset, STOP/RP2, radio loss, drive feedback, CAN bindings and safety wiring.
The two-module control assignment is confirmed; the default four-module PAIL
mapping still permits capture only until verified.

The [robot description](src/mobotic_description/README.md), wheel odometry and
`odom -> base_link` are implemented. ROS/RViz checks, covariance/slip calibration
and physical scanner extrinsic calibration remain required. `map -> odom`
localization is outside this stack. Unknown STO bit mappings remain raw
telemetry and do not constitute decoded safety confirmation.

## Container deployment

An ARM64/AMD64 Ubuntu 24.04 deployment is available in [deploy/README.md](deploy/README.md).
It builds a ROS 2 Jazzy runtime image, identifies the two USB-to-CAN adapters by
their unique USB serial numbers, configures platform `can0` and listen-only
battery `can1`, and starts the hardware stack under a single systemd supervisor.
Adapter loss and host-health faults keep ROS alive and latch ERROR with motion
inhibited. Recovery requires explicit reset rather than a whole-stack restart.
See the deployment guide for installation,
rollback, diagnostics, GUI tools, and hardware acceptance tests.
