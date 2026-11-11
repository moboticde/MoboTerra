# mobotic_bringup

Integrated launch for the current MoboTerra stack, with two supervised motion paths:

- Velocity: operator/external autonomy -> supervisor -> kinematics -> supervisor -> driver.
- External steering position and traction-current commands: autonomy -> supervisor -> driver (`KINEMATICOFF`, mode 2; miControl traction).

Both paths converge into final wheel_modules/command arrays. Internal operation
commands separately manage global enable/disable and explicit reset transactions.
The temporary F310/F710 USB profile uses Back (standardized button 4) to toggle
MANUAL/AUTONOMOUS; face buttons are unused. Terminal requests use
`vehicle/set_mode`, the only API accepting KINEMATICOFF. Both first stop and verify standstill, then
switch the sole selected mode. The driver enforces exclusive setpoint routing
using fresh `vehicle/mode`; KINEMATICOFF needs no extra opt-in setting.
The default `operator` node is `remdevice_control`: PAIL/Ecobox node 9, 500 kbit/s,
on the drive CAN bus. Its unverified mapping permits capture only. Follow the
[REMdevice commissioning guide](../../docs/remdevice_commissioning.md).
Selecting `operator.source: usb` starts the configured USB driver: generic
`joy_node` for PAIL, or `game_controller_node` for standardized Logitech ordering.
The PAIL USB mapping must be verified separately before enabling motion.
See [the complete temporary control map](../mobotic_manual_control/README.md).

## Hardware launch

CAN receivers use a 100 ms maximum blocking receive wait and suppress ordinary
idle-timeout warnings. Frames are delivered as they arrive; this does not slow
CAN traffic or change acquisition timestamps. Real SocketCAN receive errors,
CAN error-frame monitoring and battery/operator/drive freshness watchdogs remain
enabled. During charging or a B2 wait, use the supervisor's `WAITING` instruction
to identify the actual blocker instead of treating a short quiet bus interval
as a fault.

The PAIL input uses separate receive-only channels: `operator_rx/can_rx` for
TPDO1/2 and `operator_events/can_rx` for EMCY, SDO health replies and heartbeat.
Each has its own filtered SocketCAN reader and bounded ROS history, so diagnostic
bursts cannot displace control PDOs. Both preserve acquisition timestamps and
CAN-error reception. A shared drive/operator interface still has one CAN sender;
the extra event reader introduces no CAN transmissions.

After building and sourcing the workspace:

```bash
ros2 launch mobotic_bringup moboterra.launch.py \
  stack_type:=hardware robot_name:=moboterra \
  can_interface:=can0 battery_can_interface:=can1
```

One launch starts the driver, kinematics, odometry, supervisor and platform diagnostics.
By default it also starts the drive SocketCAN bridge, a separate receive-only
battery CAN receiver, the selected operator provider,
FlexiSoft bridge and safety monitor, Vanguard battery monitor and both
SICK scanner nodes and the generic scanner-safety adapter. Autonomy is not launched.
Wheel odometry does publish `odom -> base_link` on the namespaced `tf` topic.
The shared [description](../mobotic_description/README.md) starts by default,
including joint/static TF and CAD-envelope corner frames. Use start_description:=false
if an external owner supplies it; do not start two robot_state_publishers.
Physical startup waits for supervisor readiness, initializes modes/PDOs/ramps
and enables with zero targets before requiring motion feedback. Actual fresh
feedback then permits motion. B2 requests reset and zero-target re-enable.
See [drive startup](../../docs/drive_startup.md); physical acceptance remains
outstanding without hardware.

Required external packages: `ros2_socketcan`, `joy`, `sick_safetyscanners2`.
Bringup does not configure the OS CAN interfaces or PC network interface.
Match the selected profile: drives use `can0`, battery telemetry uses `can1`.
The four-module reference
uses 500/500 kbit/s. Configure the battery adapter in listen-only mode in the OS.
Drive frames remain on `can_rx`/`can_tx`. Battery frames are isolated on
`battery_can/can_rx`; the battery monitor remaps its input there. No battery
sender is launched, and this does not implement operating-consent/HV control.
Several nodes respawn automatically, including the supervisor and driver.
The FlexiSoft bridge exits on unexpected TCP worker failure and respawns after
1 s; normal client disconnects are handled without restarting the process.

The hardware bridge explicitly uses `filters=0:0,#1FFFFFFF`: all normal
standard/extended traffic plus every SocketCAN error class. Error-frame reception
must be enabled separately from data reception; otherwise the driver's CAN-error
counter cannot observe kernel-reported errors. This uses the bridge's documented
[filter implementation](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan/src/socket_can_receiver.cpp).
If `start_socketcan:=false`, configure the external drive bridge equivalently and preserve
CAN acquisition timestamps. This changes diagnostics reception, not drive enable,
fault reset, CAN bitrate or automatic bus recovery.

## Shared configuration

Use `ros2 run mobotic_bringup check_readiness --platform-config /etc/moboterra/platform.yaml`
for a read-only report of live startup/feedback, battery and operator blockers.
Add `--offline` to validate configuration without ROS or CAN access. See the
[software readiness review](../../docs/software_readiness.md) for scope and
target-machine verification. `ready_to_drive` includes all hardware/health
prerequisites; it does not require an active motion command.

All bundled YAML now belongs to [mobotic_config](../mobotic_config/README.md).
`platform_config` selects the physical profile for every component; use the same
profile when launching the standalone description. Launch resolves and validates
configuration before returning any ROS node actions. Topics and control routing
have not changed. `check_config` is read-only and does not configure OS interfaces.

## Launch options

| Argument | Default | Meaning |
|:--|:--|:--|
| `stack_type` | `hardware` | `hardware` or protocol-test `virtual` profile |
| `platform_config` | installed `mobotic_config/config/platform.yaml` | Shared hardware profile |
| `robot_name` | `ROBOT_NAME` or `auto` | Explicit name (including empty root) wins; `auto` uses the profile name |
| `can_interface` | `auto` → can0 | Drive SocketCAN interface |
| `battery_can_interface` | `auto` → can1 | Receive-only battery receiver; may share the drive interface with matching bitrates and listen-only disabled in the profile |
| `start_socketcan` | `true` | Start drive CAN bridge only |
| `start_battery_socketcan` | `true` | Start receive-only battery CAN receiver when `start_battery=true` |
| `start_manual_interface` | `true` | Start joystick and teleop/manual-control node |
| `start_safety` | `true` | Start hardware FlexiSoft monitoring |
| `start_battery` | `true` | Start hardware Vanguard monitor |
| `start_scanners` | `true` | Start hardware scanners and scan-health monitoring |
| `operator_can_interface` | empty | Optional receive-only PAIL CAN input in virtual mode, isolated from simulated drive traffic |
| `start_odometry` | `true` | Start measured wheel odometry in either profile |
| `publish_odom_tf` | `auto` → true | Publish odom-to-base TF; disable if an external estimator owns it |
| `front_scanner_ip` | `auto` → 192.168.1.5 | Front scanner address |
| `rear_scanner_ip` | `auto` → 192.168.1.6 | Rear scanner address |
| `scanner_host_ip` | `auto` → 192.168.1.250 | PC NIC address; verify on the actual host |

Overlay node policies using `driver_parameters_file`,
`kinematics_parameters_file`, `manual_parameters_file`,
`safety_parameters_file`, `battery_parameters_file`,
`supervisor_parameters_file`, `odometry_parameters_file`, `scanner_parameters_file`
or `diagnostics_parameters_file`. These default to empty (no overlay). Use the
`/**: ros__parameters:` format; matching old hardware entries are accepted, but
conflicting hardware is rejected—edit `platform_config` instead. `publish_odom_tf:=auto`
honors `publish_tf` from effective odometry policy; explicit `true`/`false` wins.
Disabling safety/battery nodes does not bypass supervisor readiness checks.
`start_battery_socketcan:=false` leaves the battery monitor running; an external
receiver must then publish timestamped frames on `/moboterra/battery_can/can_rx`.
The two transport switches are independent. Disabling battery monitoring also
disables its receiver; virtual mode starts neither physical transport.
Disabling scanners removes only scanner nodes and their diagnostic scan checks.

The four-wheel driver profile uses a 10 ms CAN heartbeat and the reference
traction ceiling of 116053 motor increments/s (~11.126 rad/s wheel speed).
Acceleration/deceleration are explicitly 32093 motor increments/s^2, calculated
for 1 m/s^2 with a 0.325 m wheel radius, 16:1 gearing and 4096 counts/revolution.
The supervisor's command ramp remains 0.5 m/s^2; controller ceilings are not
requested driving speeds. Custom wheel/scaling configurations must update both
controller profile and per-drive limits consistently.

## Odometry and scanner data

`mobotic_odometry` integrates measured `vehicle/measured_velocity` (derived from all configured modules),
including sideways crab motion and yaw, and publishes `/moboterra/vehicle/odom`.
The `odom -> base_link` transform carries the same pose/acquisition timestamp,
on `/moboterra/tf`. Never run another broadcaster for that transform unless
`publish_odom_tf:=false`. The CAD/joint description is still launched separately.
Covariance defaults need hardware calibration. Feedback loss stops publication;
recovery does not integrate through gaps. Unlike the control nodes, odometry does
not automatically respawn, since that would silently reset its origin.
Platform diagnostics monitors odometry freshness when `start_odometry=true`.
See [odometry behavior and reset policy](../mobotic_odometry/README.md).

Each scanner exposes `scanner/<location>/scan` (`sensor_msgs/msg/LaserScan`) and
`scanner/<location>/raw_data` (`sick_safetyscanners2_interfaces/msg/RawMicroScanData`),
where locations are `front_left` and `rear_right`. Raw data is the vendor's decoded
ROS message, not a PCAP/network-byte stream. Measurement, intrusion and application
I/O data are enabled in `mobotic_config/config/scanners.yaml`. The scanner adapter
uses evaluated output paths in raw data to publish `scanner/<location>/safety_zone_status`.
Both streams gate supervisor motion: red commands zero immediately, yellow ramps
to a 0.3 m/s cap, and clear fields remove that cap. Unknown/stale status blocks
motion. `scanner/<location>/field_data` exposes configured field geometry for
inspection; geometry is not re-evaluated in ROS. Hardware uses scanner Ethernet
telemetry; virtual bringup selects mock safety I/O explicitly. Scanner acquisition still needs verification
on the actual network; no raw-data recording or scanner extrinsic TF is added.

```bash
ros2 topic info /moboterra/scanner/front_left/raw_data -v
ros2 topic info /moboterra/scanner/rear_right/raw_data -v
ros2 topic info /moboterra/vehicle/odom -v
```

## Enable policy and commissioning

The supervisor continuously requests enable when safety and aggregate battery
state are fresh and ready with valid SOC. SOC below 20% is a warning only and
does not block startup, motion or B2 recovery. Missing/invalid telemetry, HV loss
and battery faults still request stop + disable. Enable releases hardware brakes according to the machine wiring;
independent brake feedback is unknown. Enable does not require a joystick deadman.

Motion additionally requires enabled, fresh, fault-free wheel feedback and a
valid selected command source. Manual motion requires a fresh connected/active
joystick source, but no held button in the default USB profile. Ackermann uses left steering plus right forward/reverse; crab uses only the right
stick. The first steering input owns the family until neutral and measured stop;
crab wins simultaneous input. Limits are +/-45 degrees in Ackermann and +/-90
degrees in crab. Traction and steering run concurrently; steering profiles track
individual targets without a wait-for-heading traction gate. The steering limit
reference remains fixed during each homing/authority cycle. Either L1 or R1 boosts travel speed without stacking.
Management lease loss (0.3 s) stops/disables independently of joint-command loss
(0.1 s) and decoded CAN feedback loss.

Public fault reset is `/moboterra/wheel_modules/clear_errors`, owned by the
supervisor and verified asynchronously. Driver initialization/disable services
are internal. Initialization rejects enabled drives; a diagnostic disable is
not a persistent override while the supervisor requests enable.

Verify the node IDs, gearing, geometry, limits, joystick layout, battery participant
mapping, FlexiSoft telegram and hardware stop/brakes before operating the platform.
See [hardware monitoring and secured tests](../../docs/hardware_monitoring.md).

## Virtual profile: protocol testing, not physical simulation

```bash
ros2 launch mobotic_bringup moboterra.launch.py \
  stack_type:=virtual robot_name:=moboterra start_manual_interface:=false
```

With the default profile this starts the normal driver with the same PDO configuration as hardware mode, eight local
`virtual_mobotic_drive` nodes and `mock_platform_state` publishing ready safety
and aggregate battery state. CAN topics are connected locally; no SocketCAN bridge,
physical safety/battery monitor or scanner is launched. It does not command
the physical CAN interface.

The virtual platform contains eight miControl SDO drive models: steering and
traction for every module. Feedback requires all eight joint samples. Geometry
and units come from the selected four-module configuration. The models do not
represent physical motor, brake or safety behavior.

### Drive the virtual platform with a PAIL CAN joystick

The confirmed receiver for this test is node 10 at 250 kbit/s. Use the separate
virtual profile prepared in the [command guide](../../docs/commands.md#pail-can-joystick-and-rviz-in-virtual-mode),
with simulated master node 11 and the measured PAIL axis/button mapping.
With its physical CAN interface already configured:

```bash
ros2 launch mobotic_bringup moboterra.launch.py \
  stack_type:=virtual robot_name:=moboterra_test \
  operator_can_interface:=can0 start_scanners:=false \
  platform_config:="$HOME/.config/moboterra/platform-virtual-pail.yaml"
```

The optional receiver publishes only to `operator_can/can_rx`. The operator's
CAN input/output are remapped to this separate namespace; virtual drive frames
remain on the local `can_rx`/`can_tx` topics. No physical CAN sender is launched,
so neither simulated drive commands nor operator NMT requests reach the bus.
The Ecobox must already transmit PDOs. If it is Pre-Operational, explicitly start
node 10 on the isolated joystick bus with `cansend can0 000#010A`.
The option has no effect in hardware mode or when the manual interface is disabled.

Physical scanners, FlexiSoft and batteries are unnecessary for this virtual
profile: mock readiness supplies those inputs. The generic scanner adapter
reports the mock I/O status; it does not connect to scanners. An unverified PAIL
mapping still inhibits joystick motion. See the [Docker and RViz commands](../../docs/commands.md#pail-can-joystick-and-rviz-in-virtual-mode).

## Inspect the system

When both hardware driver PDO flags are false, bringup selects
`sdo_socket_can_bridge.launch.py`: the existing SocketCAN receiver plus one
`mobotic_driver/paced_socket_can_sender`. Shared-bus PAIL traffic uses that same
sender. Every producer shares the 2 ms minimum software send gap defined once
in `mobotic_driver/can_scheduling.hpp`; PEAK adapters use the same SocketCAN path.
Its write queue age is bounded by 50 ms or half the configured command
watchdog, whichever is tighter. Passive reads use the same 100 ms window as
the driver response tracker. The PDO/SYNC and virtual transport paths retain
their own adapters. Interface overrides are compared after resolution so an
operator using the drive interface cannot create a second sender on that bus.
`can_tx_sent` and `can_tx_status` expose socket acceptance and queue statistics
without claiming physical transmission timing. See the driver README for
admission, stop priority and expiry semantics.

```bash
ros2 node list
ros2 topic list
ros2 topic echo /moboterra/vehicle/mode
ros2 topic echo /moboterra/wheel_modules/status
ros2 topic echo /moboterra/diagnostics
rqt_graph
```

The [project graph](../../docs/moboterra_rqt_graph.md) is source-derived, not a
captured live graph. Missing publishers/subscribers may change what rqt displays.


Kinematics implementations and ROMB integration are owned by
[`mobotic_kinematics`](../mobotic_kinematics/README.md). Bringup selects
`kinematics_backend:=romb` (the only supported solver) and delegates node creation to that package.
`start_ros2_four_micontrol.sh` continues to select ROMB, using the same executable
names, topics and geometry. ROMB executables now belong to `mobotic_kinematics`.

## Manual-only commissioning test

`control_mode:=manual_only` replaces `mobotic_supervisor` with
`manual_drive_control`. The default remains `supervised`. Only operator velocity
and fresh timestamp-correlated kinematics output can reach the driver; autonomy,
RP1 mode requests and direct-current control are not accepted. Driver limits,
feedback checks and watchdogs are unchanged.

Manual-only body and final wheel speeds are capped at **0.3 m/s**, including
boosted input and rotation, because this authority has no scanner-zone evidence.
Body limits apply one common scale to forward, lateral and yaw velocity. Reduced
speed therefore preserves Ackermann turning radius and crab heading, including
partial steering and reverse travel, rather than tightening a turn as speed falls.
Use supervised operation for the active robot's 1 m/s normal and 2 m/s boosted
green-zone travel. Yellow zones and Safety Disabled remain capped at 0.3 m/s.

This explicitly omits supervisor scanner and battery ROS gates. Keep the
independent hardware scanner/stop circuit working; software tests do not verify
its physical effectiveness. The robot-local wrapper preserves the original
launcher's collision, CAN and port checks and original romb kinematics:

```bash
/home/mobotic/start_ros2_manual_only.sh
```

The wrapper omits ROS scanner/safety/battery, odometry and description nodes. It
never reconfigures scanner hardware. Drives remain disabled until neutral controls
and B2 (or `wheel_modules/enable_drives`). Recovery clears faults and initializes
at zero targets, then requires acknowledged enabled feedback and 250 ms measured
joint standstill. Neutral controls are required to request B2 recovery. Once
recovery starts, stick movement is ignored: no traction/steering motion source
is authorized and it does not cancel homing or disable drives. After completion,
discarded recovery commands cannot run; return to neutral for fresh rearm before
moving. B2 is enable-only and does not cancel startup.

Brief control-loop delays over 100 ms discard pending motion without latching
drive disable. The control-gap latch uses monotonic time with a 300 ms default
(control_gap_timeout), independently of ROS timestamp jitter. Repeated ROS times
or clock discrepancies discard cached commands; up to 40 ms of ROS clock skew
(clock_skew_tolerance) is tolerated for input acquisition. Receipt-age deadlines,
command ordering and the independent 100 ms driver motion watchdog still apply.
Large clock changes can still invalidate input freshness.

Operator STOP/loss, stale or faulted drive feedback, a sustained monotonic control
gap or another mode authority still latches disable. Correct the cause, return to
neutral and press B2 again. The manual authority does not respawn or automatically
re-enable.

From the same ROS environment with `ROS_DOMAIN_ID=42`:

```bash
ros2 topic echo /moboterra/vehicle/mode
ros2 service call /moboterra/wheel_modules/disable_drives std_srvs/srv/Trigger '{}'
```

Stop with Ctrl-C. To restore supervised operation, stop the manual launch and use
`/home/mobotic/start_ros2_four_micontrol.sh`. Never run both authorities together.


## Native launcher shutdown

Both `/home/mobotic/start_ros2_four_micontrol.sh` and its manual-only wrapper now
run through `mobotic_bringup.shutdown_guard`. On Ctrl-C, SIGTERM, terminal hangup,
or launch exit, the guard first stops the authority, leaves the driver and CAN
sender alive for the authority lease to expire, then shuts down the launch group.
After its writers exit, it sends disable to every configured drive and reads back
confirmation on the same CAN interface. miControl requires `DEV_Enable 0x3004=0`
AND `DEV_Status 0x3002` enabled bit clear; ELMO requires controlword readback zero and
the actual status word's operation-enabled bit clear. Every drive is attempted
even if another fails; unverified shutdown reports an error and nonzero exit.
No enable, reset or parameter-storage command is issued by shutdown cleanup.

The guard prints `Shutdown complete: all configured drives verified disabled`
only after successful readback. This applies to the two native launch scripts;
calling `ros2 launch` directly bypasses the guard. SIGKILL, power loss, broken CAN
and an unavailable controller cannot provide verified software shutdown; the
independent hardware safety circuit remains necessary.


### PAIL diagnostics during an accepted homing sequence

`operator/homing_state` supplies timestamped physical interlocks independently
of the command decoder's EMCY invalidation. It is used only during an already
accepted B2 reset/homing sequence in MANUAL, in supervised and manual-only
operation. Initial B2 admission and normal driving still use `operator/state`.
The observed PAIL communication diagnostics 0x8110/0x8150 with register 0x11 do
not themselves cancel homing while complete fresh physical PDOs continue to
prove neutral controls, LINK/EMGY healthy and STOP released. They remain visible
in the ordinary operator state and raw diagnostics.

STOP/E-stop, radio/link loss, stale/malformed physical inputs, non-neutral sticks,
CAN error frames, other/unknown receiver faults, drive faults or invalid feedback
still inhibit/cancel the sequence. Existing driver authority leases and homing
velocity/time limits are unchanged. The homing topic never publishes an active
motion command and is not used for autonomous or normal joystick motion.
After homing, ordinary operator readiness is required for motion again.


The geometry for these manual controls is derived from the platform profile and
shared with kinematics. Native and original romb backends both pass through
final manual envelope/alignment checks. Homing still uses the configured existing
encoder-zero reference; this does not calibrate a mechanical home switch or
prove physical wheel-angle scale. See the manual-control README for controls and
selection/release behavior.

The host launcher /home/mobotic/start_ros2_four_micontrol.sh runs the same verified
node-10 PAIL recovery as clear_joystick_error.sh after its stopped-stack/CAN
preflight and before ROS nodes start. The manual-only wrapper inherits this
behavior. Keep sticks neutral and B2 released. All eight drives must already
report disabled. A failed reset or nonzero verification aborts launch; runtime
PAIL faults still inhibit and require correction/rearm.
