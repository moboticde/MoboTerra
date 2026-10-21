# MoboTerra installation and command reference

MoboTerra exposes **operator and vehicle interfaces** for source inputs, mode
selection, readiness and fault recovery. This guide covers their implemented
commands, including AUTONOMOUS, KINEMATICOFF and the automatic SAFETY/ERROR states.

**Version check before use:** the saved operator interface has now been
integrated into the active source, including its untracked message files.
Existing containers still need to be rebuilt and installed message definitions
checked. No automatic fallback aliases for the old topic names are provided.
See the [source audit](interface_source_audit.md) and the
[PAIL capture guide](remdevice_commissioning.md).

The combined installer retains software-only setup and supports Ubuntu 24.04
ARM64/AMD64 with mounted configuration/state. Its mocked Linux deployment tests
pass; full Docker/Jazzy builds and runtime checks remain required on the target
architecture before hardware use.

All host commands below run in **Ubuntu Bash**, not Windows PowerShell. Start in
the MoboTerra repository root (the directory containing `install.sh`, `deploy/`
and `src/`). No host ROS installation is needed for the Docker workflow.

## 1. Install software without robot hardware

```bash
bash ./install.sh
```

No arguments means software-only setup. It installs missing prerequisites,
builds versioned runtime/GUI/CAD images, runs tests and isolated virtual checks,
and installs `moboterra-gui` and `moboterra-tools`. Internet access and an Ubuntu
administrator password are needed. It does not create a new hardware service or
configure CAN. It also does **not stop an existing hardware service**: this is
not a way to disable a running robot.

Equivalent explicit command, including a chosen ROS domain:

```bash
sudo bash ./deploy/install.sh --software-only --ros-domain-id 42
```

## 2. Test the installed runtime without adapters

On the Ubuntu host, load the installed software image selection, then run the
isolated smoke check. This publishes no motion commands, mounts no hardware and
does not use host networking:

```bash
MOBOTERRA_IMAGE="$(python3 -c \
  'import json; print(json.load(open("/opt/moboterra/software/release.json"))["runtime_image"])')"
sudo docker run --rm --network none \
  --env ROS_LOCALHOST_ONLY=1 --env ROS_DOMAIN_ID=142 \
  "$MOBOTERRA_IMAGE" \
  python3 /opt/moboterra-checks/virtual_smoke.py
```

Expect a successful exit and a PASS report covering eight joints, wheel status,
odometry and diagnostics. This is a software test, not hardware commissioning.

For an interactive virtual stack, use the same image variable in that terminal:

```bash
sudo docker run --rm -it --name moboterra-virtual \
  --network host \
  --env ROS_DOMAIN_ID=142 --env ROS_LOCALHOST_ONLY=1 \
  "$MOBOTERRA_IMAGE" \
  ros2 launch mobotic_bringup moboterra.launch.py \
  stack_type:=virtual robot_name:=moboterra_test \
  start_manual_interface:=false
```

Keep this terminal open. In a second Ubuntu terminal:

```bash
sudo docker exec -it moboterra-virtual \
  /usr/local/bin/moboterra-entrypoint bash
ros2 node list
ros2 topic echo --once /moboterra_test/wheel_modules/status
```

The second terminal is now a sourced **container shell**; `exit` leaves it.
Domain 142 and namespace `/moboterra_test` separate simulated feedback from the
default hardware stack (domain 42, namespace `/moboterra`). On an Ubuntu desktop:

```bash
moboterra-gui env ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 rqt_graph
```

Run that GUI command on the **host**, not inside the container shell. Stop the
foreground virtual stack with Ctrl+C. Virtual tests do not verify actual CAN,
scanner protective stops, brakes, encoder recovery or physical E-stop behavior.

### PAIL CAN joystick and RViz in virtual mode

After source changes, rebuild the software images with
`sudo bash ./deploy/install.sh --software-only`. This does not start the hardware
service. Reload `MOBOTERRA_IMAGE` from section 2 after a successful installation.

The confirmed PAIL/Ecobox settings for this virtual test are **node 10 at
250 kbit/s**. Drives, safety and battery readiness are virtual. Use a CAN adapter
connected to the Ecobox separately from physical drives. The installed hardware
profile still describes the planned shared node-9/500-kbit/s bus; keep a separate
virtual profile so this test does not change that installation.

Keep the physical robot service stopped during this joystick test. Virtual drive
frames are exchanged only inside `/moboterra_test/can_rx` and `can_tx`; the
physical joystick has a separate receive-only bridge in `operator_can`. There is
no CAN sender connecting either virtual drive commands or mocked readiness to
the physical bus. The virtual ROS domain is 142, separate from the normal
hardware domain in the installed configuration.

For a new setup, create the virtual profile once (Python and PyYAML are required).
An existing file is preserved by exclusive creation:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml

profile = yaml.safe_load(Path('/etc/moboterra/platform.yaml').read_text())
profile['can']['master_node_id'] = '0x0B'  # Simulated master, node 11.
profile['can']['drives']['bitrate'] = 250000
profile['operator']['source'] = 'remdevice'
profile['operator']['mapping']['node_id'] = 10
output = Path.home() / '.config/moboterra/platform-virtual-pail.yaml'
output.parent.mkdir(parents=True, exist_ok=True)
with output.open('x') as stream:
    stream.write('# VIRTUAL PAIL TEST ONLY; do not deploy as hardware configuration.\n')
    yaml.safe_dump(profile, stream, sort_keys=False)
print(output)
PY
```

Node 11 avoids the configuration's node-10 master collision. Its heartbeat stays
inside the virtual CAN topics; the physical receiver remains node 10. Configure
the adapter on the isolated joystick bus (these commands assume it is `can0`):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000 listen-only off
sudo ip link set can0 up
ip -details link show can0
```

The virtual launch does not configure the host interface. Its optional operator
receiver has no physical CAN sender, so simulated drive traffic stays inside ROS.
The virtual profile must contain the measured PAIL axis/button mapping.
The supplied empty `mapping.verified: false` profile permits inspection but
intentionally cannot move even the virtual platform. Follow the
[PAIL capture guide](remdevice_commissioning.md#capture-the-customized-control-assignments)
to determine the real axes/buttons; do not substitute assumed gamepad indices.
The receive-only capture tool also runs on this host without ROS:

```bash
PYTHONPATH=src/mobotic_manual_control python3 -m mobotic_manual_control.remdevice_trace \
  --interface can0 --node-id 10 --output pail.jsonl --label neutral --seconds 8
```

Once the interface is up and the receiver is transmitting PDOs, launch the
virtual stack from a host terminal:

```bash
MOBOTERRA_IMAGE="$(python3 -c \
  'import json; print(json.load(open("/opt/moboterra/software/release.json"))["runtime_image"])')"
sudo docker run --rm -it --init --name moboterra-virtual \
  --network host --cap-drop ALL --security-opt no-new-privileges:true \
  --env ROS_DOMAIN_ID=142 --env ROS_LOCALHOST_ONLY=1 \
  --volume "$HOME/.config/moboterra/platform-virtual-pail.yaml:/etc/moboterra/platform.yaml:ro" \
  --volume "$PWD/deploy/config:/etc/moboterra/virtual-policy:ro" \
  "$MOBOTERRA_IMAGE" \
  ros2 launch mobotic_bringup moboterra.launch.py \
  stack_type:=virtual robot_name:=moboterra_test \
  platform_config:=/etc/moboterra/platform.yaml \
  start_manual_interface:=true start_scanners:=false operator_can_interface:=can0 \
  driver_parameters_file:=/etc/moboterra/virtual-policy/virtual-pi-driver.yaml \
  supervisor_parameters_file:=/etc/moboterra/virtual-policy/virtual-pi-supervisor.yaml \
  manual_parameters_file:=/etc/moboterra/virtual-policy/virtual-pi-manual.yaml
```

The operator receives physical frames on `operator_can/can_rx`; its outgoing
`operator_can/can_tx` has no physical sender. Its start-receiver service and
automatic NMT therefore cannot start the physical Ecobox in this mode. If the
receiver is Pre-Operational, explicitly start node 10 on the isolated joystick
bus with `cansend can0 000#010A`. This sends NMT Start only to node 10; it does not
change its node ID, bitrate or stored configuration. Never use a broadcast start
for this test. Expect TPDO1 `18A`, TPDO2 `28A`, TPDO3 `38A`, heartbeat `70A` and
EMCY `08A` for node 10.

In another **host desktop** terminal, start RViz:

```bash
MOBOTERRA_GUI_DRI=1 moboterra-gui env ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=0 \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
  FASTRTPS_DEFAULT_PROFILES_FILE=/opt/moboterra-checks/virtual-fastdds.xml \
  rviz2 -d /opt/moboterra-checks/virtual-pail.rviz --ros-args \
  -r /tf:=/moboterra_test/tf -r /tf_static:=/moboterra_test/tf_static
```

The saved view follows `base_link`, shows the robot and ground grid, and disables
the odometry arrows/covariance overlay that can obscure the platform. Its fixed
frame is `odom`. To configure an empty RViz window manually, add **RobotModel**,
choose **Topic** as its description source and
`/moboterra_test/robot_description` as the topic; use
**Transient Local** durability. Add **Odometry** with
`/moboterra_test/vehicle/odom` if desired. Mock scanner readiness is available,
but no laser scans or obstacle world are simulated. Keep controls neutral until
MANUAL readiness and complete joint feedback are established, then move the
PAIL sticks. Missing feedback still blocks motion; removing scanner hardware
does not bypass that gate.

The left paddle steers the wheels, including while stationary. The right stick
commands forward/reverse and left/right crab translation; with the left paddle
centred it does not request yaw. Keep both controls centred for initial rearm.

The GUI command selects loopback-only UDP discovery because RViz and the runtime
use separate container shared-memory namespaces and user IDs. With Jazzy,
`ROS_LOCALHOST_ONLY=1` installs a shared-memory transport even when
an XML transport is set, so both discovery overrides above are
needed for the GUI. The supplied XML disables built-in transports and permits
only `127.0.0.1`. The runtime stays localhost-only on its separate domain.
`MOBOTERRA_GUI_DRI=1` enables the Pi GPU when `/dev/dri` is available. With software
rendering, set RViz's **Frame Rate** to 10. After a transient startup
fault, centre the controls and press B2 to request the supervisor's normal reset;
motion remains blocked if feedback is still stale.

The optional `virtual-pi-*` overlays above run control and synchronous feedback at
20 Hz to reduce CPU load when RViz shares the Pi. They retain the default
watchdog, input and feedback deadlines. Omit the mount and three policy arguments
to use the default 50 Hz policy. Use these overlays only for `stack_type:=virtual`.

## 3. Configure the real adapters without starting motion

Secure the platform and make the independent hardware stop available first.
The software is not hardware-commissioned or safety-rated. Do not start a second
hardware launch while the supervised container is running.

```bash
sudo bash ./deploy/install.sh --no-start
```

Follow the prompts, connecting one adapter at a time to identify its unique USB
serial. `--no-start` still requires real adapter serials; use `--software-only`
when they are unavailable. It installs the hardware release and leaves the
service stopped and disabled, including on a previously configured installation.

For unattended provisioning, replace both placeholders with actual unique serials:

```bash
sudo bash ./deploy/install.sh --no-start \
  --platform-serial PLATFORM_SERIAL \
  --battery-serial BATTERY_SERIAL \
  --ros-domain-id 42
```

The host supervisor assigns the platform adapter to `can0` at
`can.drives.bitrate` and the battery adapter to `can1` at
`can.battery.bitrate`, listen-only. Both values come from the selected
`/etc/moboterra/platform.yaml`; legacy env `CAN_BITRATE` is ignored. It owns link naming,
configuration and container lifecycle; do not bring CAN up in the container.
There is no automatic USB-port fallback when serials are missing or duplicated.

Scanner/safety Ethernet addresses and physical safety wiring must be configured
separately. The installer does not change them. Review commissioning checks in
[the deployment manual](../deploy/README.md#acceptance-checks) before enabling motion.

## 4. Hardware start, stop and boot behavior

These commands are on the **Ubuntu host**, after hardware configuration exists:

```bash
sudo moboterra-ctl status
sudo moboterra-ctl can
sudo moboterra-ctl start
sudo moboterra-ctl stop
sudo moboterra-ctl restart
```

Start/restart can enable drives when readiness gates allow it. A ROS/container
stop is not a physical emergency stop. `start` starts this session only; it does
not enable boot startup after `--no-start`. To opt into boot startup explicitly:

```bash
sudo systemctl enable --now moboterra-supervisor.service
```

To stop now and prevent automatic startup on later boots:

```bash
sudo systemctl disable --now moboterra-supervisor.service
```

The full guided command `sudo bash ./deploy/install.sh` both installs and
enables/starts the hardware service. Use it only when the secured robot is ready.
Missing configured adapters now keep ROS running with motion inhibited in ERROR.
The managed deployment feeds host fault state into the ROS supervisor; recovery
requires cleared causes and an explicit supervisor reset. Process discovery and
a Docker healthy state alone do not prove that the robot is ready or safe to move.

## 5. Logs, CAN and adapter replacement

```bash
sudo moboterra-ctl logs --lines 200
sudo moboterra-ctl logs --follow
sudo docker logs --follow --tail 200 moboterra-robot
sudo journalctl -u moboterra-supervisor.service -n 200 --no-pager
ip -details -statistics link show can0
ip -details -statistics link show can1
```

`moboterra-ctl logs --follow` follows the **host supervisor journal**, not live
container output. Use `docker logs --follow` for ROS stdout/stderr. Ctrl+C exits
a log follower without stopping the robot. ROS files are under
`/var/lib/moboterra/ros-logs`; Docker and ROS log retention are bounded.

To inspect the serial for a connected CAN interface (substitute its actual
current name if it is not `can0`):

```bash
udevadm info --query=property --path=/sys/class/net/can0 \
  | grep -E 'ID_SERIAL_SHORT|ID_PATH'
```

For replacement adapters, stop first, rerun guided identification, then review
configuration before starting again:

```bash
sudo moboterra-ctl stop
sudo bash ./deploy/install.sh --reconfigure-can --no-start
sudo moboterra-ctl can
# Only after review and safe hardware commissioning:
sudo systemctl enable --now moboterra-supervisor.service
```

The current service is `moboterra-supervisor.service`, not `robot-can.service`.
Configuration is `/etc/moboterra/moboterra.env`, not `/etc/default/robot-can`.

## 6. Inspect the running ROS system

On the host, enter the supervised hardware container:

```bash
sudo docker exec -it moboterra-robot \
  /usr/local/bin/moboterra-entrypoint bash
```

The entrypoint sources ROS Jazzy and the installed workspace. Plain `docker
exec ... bash` does not automatically source either environment. Inside this
**container shell**, first verify that this image contains the interface update:

```bash
ros2 interface show mobotic_interfaces/msg/OperatorState
ros2 interface show mobotic_interfaces/msg/VehicleModeState
ros2 interface show mobotic_interfaces/msg/WheelModuleCommandArray
```

`VehicleModeState` must include AUTONOMOUS=1, KINEMATICOFF=2, SAFETY=3,
ERROR=4 and `selected_mode`. If `OperatorState` is missing or the state contains
only AUTO_VELOCITY/AUTO_DIRECT, stop here: this is the older built image.
Topic discovery alone is insufficient when a provider has not been launched.
For an updated image, use:

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 topic info --verbose /moboterra/joint_states
ros2 topic echo --once /moboterra/operator/state
ros2 topic echo --once /moboterra/vehicle/mode
ros2 topic echo --once /moboterra/wheel_modules/status
ros2 topic echo --once /moboterra/safety/state
ros2 topic echo --once /moboterra/safety/io_state
ros2 topic echo --once /moboterra/battery/system_state
ros2 topic hz /moboterra/joint_states
ros2 topic echo --once /moboterra/vehicle/measured_velocity
ros2 topic echo --once /moboterra/vehicle/odom
```

Some commands wait for a publisher; use Ctrl+C to leave them. Default robot
namespace is `/moboterra`; adapt it when launching a differently named robot.
Additional useful feedback: `/moboterra/battery/state`,
`/moboterra/scanner/front_left/scan` and
`/moboterra/scanner/rear_right/scan`. Inspect types with `ros2 topic list -t` rather than
assuming all status messages are standard ROS messages.

To run one ROS command directly from the host:

```bash
sudo docker exec moboterra-robot \
  /usr/local/bin/moboterra-entrypoint \
  ros2 topic echo --once /moboterra/vehicle/mode
```

### Current modes and services

The mode state machine stays **inside `mobotic_supervisor`**; `vehicle/mode` is
its published state, not a separate controller node. Only 0, 1 and 2 can be
selected. A mode request does not bypass readiness gates or command motion.

| Value | Mode | Selection and command path |
| --- | --- | --- |
| 0 | MANUAL | Selectable; `operator/cmd_vel` through kinematics |
| 1 | AUTONOMOUS | Selectable; `autonomy/cmd_vel` through kinematics |
| 2 | KINEMATICOFF | Selectable; `autonomy/joint_setpoints`, bypassing kinematics |
| 3 | SAFETY | Automatic confirmed safety-chain stop; cannot be selected |
| 4 | ERROR | Automatic required-system fault, latched until verified reset |
| 255 | UNKNOWN | Initial readiness/monitoring not established |

AUTO_VELOCITY=1 and AUTO_DIRECT=2 remain numeric aliases, not additional modes.
ERROR has display precedence over SAFETY; `safety_stop_active` still exposes a
simultaneous stop. `selected_mode` retains the intended 0/1/2 control mode.
`current_mode` is the effective state. Read `motion_permitted`,
`transition_in_progress`, `error_active`, `reason_codes`, `readiness_known`,
`ready_to_drive` and `drives_enabled` before interpreting an operating state.

Inside the sourced container shell, inspect the request fields first:

```bash
ros2 interface show mobotic_interfaces/srv/SetVehicleMode
ros2 service call /moboterra/vehicle/set_mode \
  mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 0}'
```

For AUTONOMOUS or KINEMATICOFF, request the corresponding value on secured
hardware or the isolated virtual stack. These are alternative requests, not a
sequence to run automatically:

```bash
# AUTONOMOUS: external body velocity
ros2 service call /moboterra/vehicle/set_mode \
  mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 1}'
# KINEMATICOFF: external steering position and traction velocity
ros2 service call /moboterra/vehicle/set_mode \
  mobotic_interfaces/srv/SetVehicleMode '{requested_mode: 2}'
ros2 topic echo /moboterra/vehicle/mode
```

The service returns `accepted`, `transition_started`, `current_mode` and
`message`. Its `current_mode` is the selected control mode in the callback;
use the topic for authoritative effective SAFETY/ERROR/UNKNOWN state and
transition completion. `/moboterra/operator/mode_request` accepts timestamped
requests for 0/1 only. Use the service for mode 2. Requests for 3/4/255 are rejected.

A transition stops the outgoing path and verifies fresh measured standstill
for 0.25 s. A 10 s timeout latches ERROR. In KINEMATICOFF, operator/autonomy
velocity candidates and kinematics output have no motion effect. Returning to
a velocity mode requires the same verified transition and a new command.

### Fault recovery and odometry reset

Investigate `reason_codes` and diagnostics and remove the cause first. On a
secured, stationary robot, the **supervisor-owned** reset request is:

```bash
ros2 service call /moboterra/wheel_modules/clear_errors \
  std_srvs/srv/Trigger '{}'
ros2 topic echo /moboterra/vehicle/mode
```

`success: true` means **accepted**, not completed. Follow `reset_in_progress`,
`reset_id`, `reset_result` and `error_active`; completion requires fresh matching
driver acknowledgements, standstill and cleared root faults. A 5 s timeout
leaves the fault latched. Reinitializing/enabling does not implicitly clear it.
Successful recovery clears cached motion and still requires a fresh command.

A confirmed safety stop clears without a fault reset only if no separate fault
is latched. Motion then requires a new command. Manual operation also requires
neutral controls; if configured, release and press the deadman again. The USB
profile has no deadman assigned by default.

Odometry has a separate standstill-guarded reset; this is not an encoder reset:

```bash
ros2 service call /moboterra/vehicle/reset_odometry \
  std_srvs/srv/Trigger '{}'
```

The remaining driver commissioning services are internal, not normal operator
controls. Both use `std_srvs/srv/Trigger`: `internal/driver/initialize_all_drives`
and `internal/driver/disable_all_drives`. The old public enable/clear service
commands are not part of the updated contract. Do not use initialization or
internal disable as fault-reset or persistent stop workarounds.

The external motion inputs are `operator/cmd_vel`, `autonomy/cmd_vel` and
`autonomy/joint_setpoints` under the robot namespace. Do not publish directly
to internal driver/kinematics paths to bypass arbitration.
`wheel_modules/command` now carries the **single final motion array**, published
only by the supervisor. Management/enable/reset transactions moved to internal
`wheel_modules/operation_command`. Kinematics publishes a candidate back to the
supervisor, never directly to the driver. These changes are not simple renames.

### Operator state and hardware feedback

`operator/state` reports physical/source inputs, not the vehicle operating mode.
`estop_pressed` means the physical operator-device button; read it only together
with `estop_state_known`. A USB gamepad reports the E-stop as unknown, not a
verified released safety button. Raw `joy` remains `joy`. The vehicle's mode
aggregates operator, safety-chain and system-fault conditions separately.

`joint_states` is the authoritative eight-joint measurement: steering and wheel
position [rad], steering and wheel velocity [rad/s]; `effort` is empty.
`wheel_modules/status` contains per-module `steering` and `traction` drive
status, with separate `motor_current` [A] and `current_known`, status/fault/mode
fields and acquisition stamps. It also exposes `position_valid`,
`position_reason`, freshness, enable, reset, brake and STO known flags.
Unknown values are not zero-valued proof of health.

The updated driver's **physical** profile presently inhibits enable and
authoritative joint publication until the retained Elmo position reference and
scale are commissioned. It reads traction position through Elmo (startup SDO
and cyclic position PDO), not a direct encoder connection. Only virtual launch
sets `virtual_position_source=true`; never use it on physical CAN. A persistent
directory/helper alone does not recover unpowered wheel movement.

Scanner field monitoring is available to autonomy as well as the supervisor:

```bash
ros2 topic echo --once /moboterra/scanner/front_left/safety_zone_status
ros2 topic echo --once /moboterra/scanner/rear_right/safety_zone_status
ros2 topic echo --once /moboterra/scanner/front_left/diagnostics
ros2 topic echo --once /moboterra/scanner/rear_right/diagnostics
```

Check communication and known flags. Field/OSSD state comes from the mapped
safety-controller I/O; field-set number is currently unknown. Protective stop
and monitoring failure are different conditions. LaserScan does not establish
protective-stop permission. See the [ROS interface manual](ros_interface_manual.md).

## 7. GUI tools and live graph

On the **host desktop**, as the logged-in user:

```bash
moboterra-gui rviz2
moboterra-gui rqt
moboterra-gui rqt_graph
```

X11/XWayland, DISPLAY and a readable Xauthority file are required. Do not prefix
the launcher with sudo; it handles Docker authentication. In rqt_graph, refresh
and adjust hidden-topic filters to capture the **actual running** graph. A
source-derived architecture figure is not a live graph capture.

In the updated bringup, `start_description` defaults to `true`, so
robot_state_publisher is integrated. Do not start a duplicate. RViz needs the
namespaced TF topics:

```bash
moboterra-gui rviz2 --ros-args \
  -r /tf:=/moboterra/tf -r /tf_static:=/moboterra/tf_static
```

For the older checkout only, description was a separate optional launch. Verify
the running nodes before applying instructions from that version.
Software GL is the default; optional DRI access is enabled with
`MOBOTERRA_GUI_DRI=1 moboterra-gui rviz2` on compatible hosts.

## 8. Edit code, pull updates and deploy a tested release

Yes, code is editable. Edit the repository on the host, not files inside the
running container: the deployed workspace is a built image, not a live source
mount. Keep local changes committed or backed up. Before pulling:

```bash
git status --short
# Continue only when your changes are safely handled and the checkout is clean:
git pull --ff-only
```

Do not blindly apply/drop an existing stash. The update from stash `a87fcd5`
and its required untracked files is already integrated into this source tree;
the stash is retained as a backup. Do not apply it a second time.
Verify section 6's message definitions in the rebuilt container before testing.

Pulling or editing source does **not** update an existing Docker image. After
reviewing changes, rebuild and run the software tests from the repository root:

```bash
bash ./install.sh
```

Then run the isolated virtual check in section 2. On an existing hardware
installation this software-only build does not select the new image for the
hardware service. During a secured maintenance window, deploy it separately:

```bash
sudo moboterra-ctl stop
sudo bash ./deploy/install.sh --no-start
# Review configuration and commissioning results before startup:
sudo systemctl enable --now moboterra-supervisor.service
sudo moboterra-ctl status
sudo moboterra-ctl logs --lines 200
```

The saved updated Compose file mounts `/etc/moboterra` read-only in the container
and passes `/etc/moboterra/platform.yaml` as `platform_config`. It also mounts
`/var/lib/moboterra/position-state` as writable state storage. Policy overrides
must name existing files under `/etc/moboterra`; their env keys include
`DRIVER_PARAMETERS_FILE`, `SUPERVISOR_PARAMETERS_FILE` and
`MANUAL_PARAMETERS_FILE`. Code changes still need an image rebuild. Platform or
policy edits need validation and a supervised restart.

Those mounts are **not present in the pulled checkout's Compose file**. Until
deployment changes are integrated, edit the checkout YAML and rebuild/redeploy;
editing a host file alone has no effect. Verify the installed container mounts
and image rather than inferring them from a directory's existence:

```bash
sudo docker inspect moboterra-robot \
  --format 'image={{.Config.Image}} mounts={{json .Mounts}}'
```

The installed software release's `release.json` records its runtime image;
`software.env` contains GUI/tool selections and the ROS domain, not a
`MOBOTERRA_IMAGE` entry. Installed hardware settings/image selections are in
`/etc/moboterra/moboterra.env`; software-only selections are in
`/etc/moboterra/software.env`. GUI prefers hardware settings when present;
offline tools use software settings. Do not store secrets in these files.

If a previous installed hardware release exists, and rollback is safe:

```bash
sudo moboterra-ctl rollback
```

Rollback swaps the installed current/previous image selections and restarts
hardware. It does not revert Git source, firmware, adapter identity or
calibration. Inspect logs and recheck hardware after either upgrade or rollback.

## 9. Offline CAD preview

On the host, in a writable output directory:

```bash
moboterra-tools
```

This writes `moboterra-preview.png` in the current directory. To inspect a
provided STEP assembly, place it in that same directory first:

```bash
moboterra-tools python /opt/moboterra-tools/inspect_scanner_datums.py \
  MoboTerra_assembly.step
```

The tools container uses the caller's user ID, mounts the current directory and
has no network. The installer does not supply the STEP source file.
