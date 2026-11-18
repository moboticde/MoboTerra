# MoboTerra customer operating manual

Documentation revision 17 November 2026

## 1 System and operating scope

This manual explains how to install, start, operate, stop and maintain the current
MoboTerra four-module control stack. Installation and configuration are tasks for
the customer's qualified service technician. Operators use the approved radio
controls and read the reported readiness and fault instructions.

The target is the Ubuntu robot computer. ROS, ROMB and RViz run inside Docker; a native ROS installation is not required. All host commands in this manual run
from the project root, the directory containing `README.md`. The Windows project
folder is a development and transfer location. It is not the physical robot runtime.

| Item | Supplied setting |
| --- | --- |
| Operating system and ROS | Ubuntu 24.04; ROS 2 Jazzy |
| Wheel modules | front left, front right, rear left and rear right |
| Drive arrangement | Four miControl steering drives and four miControl traction drives |
| CAN | One shared transmitting link, `can0`, 500000 bit/s |
| Receiver and master | PAIL receiver node 10; CAN master node 11 |
| Kinematics | Original ROMB solver and target emulator included in the Docker image |
| ROS identity | Namespace `/moboterra`; domain 42 |
| Scanner addresses | Host `192.168.1.200`; front left `192.168.1.5`; rear right `192.168.1.6` |
| Boot service and container | `moboterra-current-stack.service`; `moboterra-current-stack` |

The software supports manual driving and inputs from an external autonomy
application. It does not supply a route planner, autonomous navigation or a
safety-rated controller. Independent emergency-stop, brake and STO functions
must remain effective independently of ROS and Docker.

**Motion during enable:** a B2 enable request can turn the steering wheels to
the nearest equivalent home angle. Returning the controls to neutral can also
return steering home once traction standstill is established. Keep the full
steering sweep clear, including when no traction command is present.

The settings above describe the project source. They do not certify a delivered
machine. Complete the target checks in section 9 before operational handover.

## 2 Install Docker startup

Use the complete supplied MoboTerra project on an Ubuntu 24.04 robot computer.
Keep it in a permanent local directory. The installed service references that
directory. An internet connection is required for the first installation unless
service personnel have prepared the host packages and container image offline.

On an existing installation, stop the running stack before upgrading. A native
launch must finish its normal drive-disable shutdown. Migration from an older
managed service must be performed with that service stopped.

Open a terminal in the project root and run:

```bash
bash ./install.sh
```

Enter the Ubuntu administrator password when sudo requests it. The installer
automatically installs missing Docker Engine, Compose, Buildx and the small host
utilities needed to manage CAN and the service. The customer does not run separate
package-install commands or install ROS, QEMU or a ROMB runtime on the host.

The Docker build uses the original ROMB package supplied with this project.
It verifies the package checksum and includes the solver, its amd64 ROS libraries,
the emulator for the target CPU, ROS 2 Jazzy and RViz in the image. No native
operator account or user-home runtime directory needs to be prepared.

Before publishing the installation, the installer builds and tests the ROS
workspace, checks the bundled solver, runs an isolated virtual-stack check and
checks the installed headless launcher, RViz, Compose and service definition.
These checks do not communicate with physical CAN. A failed build or image check
does not install the new service or control command.

Successful installation installs `moboterra-ctl` and enables the current stack
for the next computer boot. It leaves the robot stack stopped now. Start it
explicitly when the machine is ready:

```bash
sudo moboterra-ctl start
```

`bash deploy/install.sh` is an equivalent entry point. Additional installation
options are for service personnel:

| Install option | Result |
| --- | --- |
| No option | Enable the next boot; leave the stack stopped now |
| `--no-start` | Install with boot startup disabled; leave stopped |
| `--software-only` | Prepare the image and CLI without registering a robot service |
| `--start` | Start ROS after installation; normal B2 and readiness checks remain active |

## 3 Start and stop the stack

At computer boot, systemd waits for Docker and the network-online target. The
preflight waits up to 60 seconds for the configured CAN interface. It configures
a DOWN physical CAN link at the saved bitrate and brings it UP. It does not
rename adapters, cycle a running bus or clear BUS-OFF automatically. An UP link
with incorrect settings or missing hardware causes startup to fail.

The service launches `start_ros2_four_micontrol.sh start_rviz:=false`. Scanners,
ROMB kinematics, robot transforms and wheel odometry remain enabled. RViz opens
separately from the logged-in desktop using the same container image.

Run these Ubuntu host commands as separate operations:

```bash
sudo moboterra-ctl start
sudo moboterra-ctl status
sudo moboterra-ctl logs --follow --lines 200
```

`start` launches the software. It does not issue the B2 drive-enable request.
Check the operator instructions in `vehicle/mode` before enabling drives.
Ctrl+C exits the log display without stopping the robot.

To stop the stack:

```bash
sudo moboterra-ctl stop
```

Shutdown withdraws motion authority and attempts to disable and verify every
configured drive over CAN. The container is given 45 seconds to stop. Read the
shutdown verification in `moboterra-ctl logs`. If disable cannot be verified, use
the machine's independent stopping procedure and investigate power and CAN.

To stop now and disable startup on later boots:

```bash
sudo moboterra-ctl disable --now
```

To restore boot startup without starting immediately:

```bash
sudo moboterra-ctl enable
```

Only one native or Docker stack may run. The shared lock and process checks
reject concurrent launches. Docker and systemd do not automatically restart an
exited stack; investigate the cause before a deliberate start. Individual ROS
nodes may respawn according to their launch policy; process presence still does
not establish drive readiness.

## 4 Manual operation and drive enable

Before operation, confirm the work area and steering sweep are clear, the
hardware stopping functions are available, and the selected configuration
matches the machine. Establish the radio link and follow the transmitter's
approved START and STOP procedure. Keep the driving controls neutral.

| Control | Configured function |
| --- | --- |
| Right stick forward and reverse | Speed input A3 |
| Left paddle left and right | Steering input A2 |
| Right stick sideways | Crab input A4 |
| RP1 | MANUAL or AUTONOMOUS request through U5; high is MANUAL, low is AUTONOMOUS |
| B1 | Manual boost through SP3; permitted by the active limits |
| B2 | Initialize and enable through SP4; not a stop button |
| RP2 | Safety selector through D5; see scanner policy below |
| STOP | Physical operator stop; confirm installed hardware response during commissioning |

1. Confirm the ROS stack is running. Read `vehicle/mode.status_message` and
   correct the reported blocker. A normal waiting state includes `B2_ENABLE_REQUIRED`.
2. With controls neutral and readiness restored, release B2 and press it once.
   A button held at launch or reconnection is not a new enable request.
3. Wait for initialization, steering alignment and feedback validation to finish.
   Traction remains zero during recovery. Observe `reset_in_progress`,
   `drives_enabled`, `ready_to_drive`, `motion_permitted` and `reason_codes`.
4. Apply a new manual command only after recovery completes and the intended
   mode is confirmed. Moving the controls can command motion without a held
   deadman button. Returning to neutral may move steering toward home.

B2 while enabled or initializing does not disable drives or cancel the request.
Use the approved hardware stopping controls for an immediate stop. Use the
service stop procedure for an orderly software shutdown.

With RP2 in Safety Enabled, both scanners' valid field information is required.
An occupied warning field limits speed to at most 0.3 m/s; an occupied protective
field inhibits motion. The configured Safety Disabled position permits a
software scanner bypass only in MANUAL and limits speed to 0.3 m/s. Both
autonomous modes require a fresh valid Safety Enabled selector. This software
selection does not change the independent safety-controller wiring.

The supplied profile has no held enable interlock and has
`require_neutral_rearm: false`. Neutral controls are still required to admit
B2 recovery. Do not assume a held deadman or a separate release-and-press rearm
step exists. Confirm the physical labels and response on the delivered radio.

## 5 Diagnose and recover faults

Read the reported cause before restarting software or pressing B2 repeatedly.
Run these commands on the Ubuntu host; ROS runs inside the container:

```bash
sudo moboterra-ctl logs --host --lines 100
sudo moboterra-ctl logs --lines 200
sudo moboterra-ctl can
sudo moboterra-ctl ros topic echo --once /moboterra/vehicle/mode
sudo moboterra-ctl ros topic echo --once /moboterra/operator/state
sudo moboterra-ctl ros topic echo --once /moboterra/wheel_modules/status
```

`logs` shows ROS container output. `logs --host` shows the systemd service
journal, including CAN preflight failures. `ros` runs the following ROS 2 command
in the current container with the correct environment. A stopped or failed
container cannot provide live ROS topics; inspect the host journal first.

| Reported condition | Required response |
| --- | --- |
| Service fails before ROS starts | Read the host journal; check Docker, saved paths, CAN name, bitrate and BUS-OFF |
| Operator unavailable or controls not neutral | Restore the radio link and STOP state; centre the controls |
| Battery communication, charging or HV blocker | Check pack power, charger connection and CAN; wait for valid pack readiness |
| Scanner data unavailable or protective field active | Check scanner power and Ethernet; clear the protective field |
| Drive status stale or startup failed | Check drive power and CAN; wait for fresh native status and correct drive faults |
| Disable not verified on shutdown | Apply the independent stopping procedure; investigate the failed drive or CAN link |

After correcting the cause, keep controls neutral, release B2 and press once.
The supervisor admits recovery only with the required external conditions and
fresh measured standstill or valid individual native disabled-state proof.
Missing feedback alone is not proof that the vehicle has stopped.

Request acceptance is not completion. The reset deadline is 20 seconds; the
driver startup attempt is bounded to 15 seconds. A failed or interrupted attempt
requires another explicit request after the cause is corrected. Old motion
commands are discarded. Do not use repeated restarts to hide an unresolved fault.

The receiver can perform a guarded one-time communication-error clear before
ROS CAN writers start. Runtime B2 operates drive enable and recovery; it does not
clear receiver communication faults. Receiver error registers are advisory in
the supplied profile, while valid fresh operator input and known STOP/link
information remain necessary for the applicable control paths.

## 6 Configuration and software maintenance

The current Docker deployment reads the following project files. It does not
use `/etc/moboterra` as the configuration authority.

| Project path | Responsibility |
| --- | --- |
| `src/mobotic_config/config/platform.yaml` | Hardware, CAN, geometry, controls, scanner addresses and shared limits |
| Component YAML files in the same directory | Driver, supervisor, operator, kinematics, odometry, safety, battery and diagnostic policies |
| `config/raspberry-pi/ros.env` | ROS domain and project-local ROS log locations |
| `config/raspberry-pi/live.rviz` | Live robot display settings |
| `runtime/docker-stack.env` | Generated image selection, project path and runtime user IDs |
| `runtime/ros/log` | ROS log files; manage disk usage during maintenance |

Only authorized service personnel should change configuration. Stop the stack
before editing. Record the original values and the reason for the change.
Central configuration and `ros.env` are mounted read-only into the container;
their edits take effect after a deliberate restart. Source code, launch scripts
and bundled display-file changes require an image rebuild.

For a software update, preserve local changes and confirm the intended branch:

```bash
sudo moboterra-ctl stop
git status --short
git branch --show-current
git pull --ff-only
bash ./install.sh
```

Run the pull only after local changes are committed or otherwise safely handled.
Do not discard configuration or apply an old stash without review. The installer
builds and checks a new image, then enables the next boot; it leaves the stack
stopped. Review the image and configuration before starting again.

Previous images and the previous image selection are retained. To select the
previous image, service personnel can use `sudo moboterra-ctl rollback`. The
command stops the stack, validates the previous image against the current
configuration without CAN access and leaves the stack stopped. Configuration
is retained rather than rolled back; review compatibility before starting again.

Confirm these current policies during commissioning: SOC below 20% is advisory;
receiver errors are advisory; FlexiSoft Ethernet is monitoring-only because
`safety.telemetry_required` is false; RP2 enables the restricted MANUAL scanner
bypass. Do not infer stronger interlocks from older documentation.

## 7 ROS monitoring and visualization

Use `sudo moboterra-ctl ros` followed by the required ROS 2 command. This runs
inside the image and needs no host ROS installation. The default namespace is
`/moboterra`; use the same ROS domain for external tools.

| Topic relative to the namespace | Information |
| --- | --- |
| `vehicle/mode` | Authoritative mode, readiness, fault reasons and recovery progress |
| `operator/state` | Operator inputs, connection, neutral and known STOP state |
| `wheel_modules/status` | Health and startup status for all four modules |
| `joint_states` | Measurements for all eight steering and traction joints |
| `vehicle/measured_velocity` | Body velocity calculated from measured wheel feedback |
| `vehicle/odom` | Wheel-derived pose and velocity |
| `battery/system_state` | Aggregate battery readiness, HV state, faults and minimum SOC |
| `scanner/front_left/safety_zone_status` | Front scanner field information and validity |
| `scanner/rear_right/safety_zone_status` | Rear scanner field information and validity |
| `diagnostics` | Component details for fault investigation |

Docker health checks look for ROS processes and discovery. A healthy container
does not prove that drives are enabled, sensors are valid, or movement is safe.
Use the status messages and physical observations together.

Position values in `joint_states` are radians. Steering velocity is rad/s;
traction velocity is wheel-contact m/s. The `effort` array is empty; electrical
motor current is reported separately in `wheel_modules/status` in amperes.
Applications must associate array entries by joint or module name.

The boot service starts robot description and odometry without a viewer.
To open the included RViz, run this from the robot's logged-in desktop:

```bash
moboterra-ctl rviz
```

The command uses the installed image, current ROS domain, live display file,
robot namespace and TF remappings. It uses X11 or XWayland and the desktop
session's Xauthority file. If Docker access needs administrator rights, the
command requests sudo for Docker. It does not require native ROS or change the
display server's access policy.

The display uses `odom` as its fixed frame. Missing valid wheel feedback can
prevent the odometry transform from appearing. The stack already owns
`robot_state_publisher`; do not start a second publisher for the same robot.
Odometry is based on measured wheels, not joystick commands. It is not absolute
localization, and its restart does not preserve an externally referenced pose.

## 8 Modes and application commands

The supervisor owns source selection and final drive commands. External
applications publish candidates and read status; they must not publish directly
to the final driver command or management topics.

| Value | Mode | Command source |
| --- | --- | --- |
| 0 | MANUAL | Operator commands through supervised kinematics |
| 1 | AUTONOMOUS | `autonomy/cmd_vel`, body velocity through supervised kinematics |
| 2 | KINEMATICOFF | `autonomy/joint_setpoints`, steering position and traction current |
| 3 | SAFETY | Automatic safety stop state; cannot be selected |
| 4 | ERROR | Automatic latched fault state; cannot be selected |
| 255 | UNKNOWN | Initial monitoring or readiness not established |

`selected_mode` identifies the requested operating source; `current_mode`
includes effective SAFETY, ERROR or UNKNOWN state. Read the readiness and motion
permission fields as well as the mode number. A mode request alone does not
publish an autonomous trajectory, but an already-running application can send
new commands once the requested source is permitted.

Use the container ROS command. Request only the required mode, for example
MANUAL:

```bash
sudo moboterra-ctl ros service call /moboterra/vehicle/set_mode \
  mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 0}'
```

Selectable values are 0, 1 and 2. Source changes stop outgoing motion and require
fresh measured standstill, normally for 0.25 seconds. A transition that cannot
complete within 10 seconds latches a fault. Acceptance in the service response
does not establish completed transition or motion readiness.

In KINEMATICOFF, `autonomy/joint_setpoints` is a stamped `sensor_msgs/msg/JointState`
containing every configured joint. Steering uses position in radians and a
nonnegative profile speed in rad/s; traction velocity must be zero and signed
traction current is carried in `effort` in amperes. This is not a wheel-speed
command mode. Current limits and overspeed checks still apply. Zero current
does not prove physical standstill.

| Service relative to the namespace | Purpose |
| --- | --- |
| `wheel_modules/enable_drives` | B2-equivalent initialization and enable; does not toggle off |
| `wheel_modules/clear_errors` | Explicit supervised fault-recovery transaction |
| `vehicle/set_mode` | Select mode 0, 1 or 2 using `SetVehicleMode` |
| `vehicle/reset_odometry` | Reset local wheel odometry only after standstill checks |

The drive and odometry services use `std_srvs/srv/Trigger`. An accepted request
must be followed through the status messages. Use `docs/ros_interface_manual.md`
for exact topic types, timestamp/frame rules and final-command fields.

## 9 Validation and customer handover

A successful software build is not a physical acceptance test. The Docker
deployment must be built and checked on the target computer. Boot, CAN, network,
drive response, scanner fields and independent stop behavior require the actual
machine. No commissioning certificate is implied by this manual.

Before customer operation, record the installed software commit, image tag,
configuration revision, machine identification, test date and responsible
commissioning personnel. Resolve the items listed under `commissioning_pending`
in the active platform file or record their approved disposition.

1. Verify all four module identities, eight drive IDs, direction, gearing,
   encoder units, mechanical travel and nearest-home steering behavior.
2. Verify the assigned CAN adapter consistently appears as `can0` at 500000 bit/s,
   with the intended devices and termination. Check both battery participants.
3. Verify the two scanner addresses, host NIC, field assignments and installed
   hardware stop/STO wiring. Test RP2 behavior in every permitted mode.
4. Verify the transmitter's axes, B1, B2, RP1, RP2, STOP and radio-loss response.
   Confirm that B2 enables only after a new press and cannot disable drives.
5. Reboot with the hardware connected. Confirm one container starts before
   desktop login, waits for B2 and retains all readiness checks.
6. Boot with CAN disconnected. Confirm the service fails with an actionable
   journal entry and no robot launch. Restore CAN and test explicit startup.
7. Interrupt required feedback and sensor communications during secured tests.
   Confirm inhibition, clear fault reporting and explicit recovery.
8. Stop the service and verify every drive-disable result. Confirm a duplicate
   native launch is rejected while Docker owns the stack.
9. Check RViz, measured odometry and external command contracts if included in
   the customer's use case. Confirm service and ROS log retrieval.

Commissioning results belong to the delivered machine's handover record.
Retain that record with the tested configuration and software identity.
