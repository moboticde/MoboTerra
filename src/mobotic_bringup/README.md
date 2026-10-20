# mobotic_bringup

Integrated launch for the current MoboTerra stack, with two supervised motion paths:

- Velocity: operator/external autonomy -> supervisor -> kinematics -> supervisor -> driver.
- External joint commands: autonomy -> supervisor -> driver (`AUTO_DIRECT`, mode 2).

Both paths converge into final wheel_modules/command arrays. Internal operation
commands separately manage global enable/disable and explicit reset transactions.
The temporary F310/F710 USB profile uses Back (standardized button 4) to toggle
MANUAL/AUTO_VELOCITY; face buttons are unused. Terminal requests use
`vehicle/set_mode`, the only API accepting AUTO_DIRECT. Both first stop and verify standstill, then
switch the sole selected mode. The driver enforces exclusive setpoint routing
using fresh `vehicle/mode`; AUTO_DIRECT needs no extra opt-in setting.
The default `operator` node is `remdevice_control`: PAIL/Ecobox node 9, 500 kbit/s,
on the drive CAN bus. Its unverified mapping permits capture only. Follow the
[REMdevice commissioning guide](../../docs/remdevice_commissioning.md).
Selecting `operator.source: usb` starts the configured USB driver: generic
`joy_node` for PAIL, or `game_controller_node` for standardized Logitech ordering.
The PAIL USB mapping must be verified separately before enabling motion.
See [the complete temporary control map](../mobotic_manual_control/README.md).

## Hardware launch

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
ROS/RViz validation remains required. Physical position continuity is not yet
commissioned, so physical enable/joint publication remain inhibited.

Required external packages: `ros2_socketcan`, `joy`, `sick_safetyscanners2`.
Bringup does not configure the OS CAN interfaces or PC network interface.
Match the reference wiring: drives use `can0`, battery telemetry uses `can1`,
both at 500 kbit/s; configure the battery adapter in listen-only mode in the OS.
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
| `battery_can_interface` | `auto` → can1 | Separate battery telemetry SocketCAN interface |
| `start_socketcan` | `true` | Start drive CAN bridge only |
| `start_battery_socketcan` | `true` | Start receive-only battery CAN receiver when `start_battery=true` |
| `start_manual_interface` | `true` | Start joystick and teleop/manual-control node |
| `start_safety` | `true` | Start hardware FlexiSoft monitoring |
| `start_battery` | `true` | Start hardware Vanguard monitor |
| `start_scanners` | `true` | Start hardware scanners and scan-health monitoring |
| `operator_can_interface` | empty | Optional receive-only PAIL CAN input in virtual mode, isolated from simulated drive traffic |
| `start_odometry` | `true` | Start measured wheel odometry in either profile |
| `publish_odom_tf` | `auto` → true | Publish odom-to-base TF; disable if an external estimator owns it |
| `front_scanner_ip` | `auto` → 10.60.20.52 | Front scanner address |
| `rear_scanner_ip` | `auto` → 10.60.20.53 | Rear scanner address |
| `scanner_host_ip` | `auto` → 10.60.20.186 | PC NIC address; verify on the actual host |

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

`mobotic_odometry` integrates measured `vehicle/measured_velocity` (derived from all four modules),
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
I/O data are enabled in `mobotic_config/config/scanners.yaml`. Scanner acquisition still needs verification
on the actual network; no raw-data recording or scanner extrinsic TF is added.

```bash
ros2 topic info /moboterra/scanner/front_left/raw_data -v
ros2 topic info /moboterra/scanner/rear_right/raw_data -v
ros2 topic info /moboterra/vehicle/odom -v
```

## Enable policy and commissioning

The supervisor continuously requests enable when safety and aggregate battery
state are fresh and ready with minimum SOC at least 20%. Otherwise it requests
stop + disable. Enable releases hardware brakes according to the machine wiring;
independent brake feedback is unknown. Enable does not require a joystick deadman.

Motion additionally requires enabled, fresh, fault-free wheel feedback and a
valid selected command source. Manual motion requires a fresh connected/active
joystick source, but no held button in the default USB profile. Left stick steers,
right stick translates/crabs, and either L1 or R1 boosts without stacking.
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

This starts the normal driver with the same PDO configuration as hardware mode, eight local
`virtual_mobotic_drive` nodes and `mock_platform_state` publishing ready safety
and aggregate battery state. CAN topics are connected locally; no SocketCAN bridge,
physical safety/battery monitor or scanner is launched. It does not command
the physical CAN interface.

Four simulators use MiControl steering and four use ELMO traction. They handle the
production SDO, enable, NMT and synchronous PDO paths; steering moves toward its
target and traction reports commanded velocity. The supplied IDs, 4096 tick
resolutions and 121/16 gear ratios match the default YAML. Keep simulator settings
aligned when using custom driver configuration. There is no physical motor,
brake or STO model. Protocol unit tests pass, but full ROS build/runtime testing
is still required before calling this a successful end-to-end motion test.
Raw ELMO safety-register warnings are expected.
The mock node publishes `battery/system_state`, not public `battery/state`.
Virtual telemetry uses the triggering SYNC's acquisition timestamp across all
eight simulated drives. Extra stop/enable requests wait for the pending sample
to finish (at most 100 ms, bounded by half the motion-feedback timeout), rather
than invalidating it mid-batch. This guard applies only to the virtual position
source; hardware keeps its configured cyclic timing.

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
