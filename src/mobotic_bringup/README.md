# mobotic_bringup

Integrated launch for the current MoboTerra stack, with two supervised motion paths:

- Velocity: manual/external autonomy -> supervisor -> kinematics -> driver.
- External joint commands: autonomy -> supervisor -> driver (`AUTO_DIRECT`, mode 2).

Wheel-module management is a third, separate path: supervisor -> driver for global
enable/disable and error clearing, never motion setpoints.
The temporary F310/F710 USB profile uses Back (standardized button 4) to toggle
MANUAL/AUTO_VELOCITY; face buttons are unused. Terminal requests use
`vehicle/set_mode`, the only API accepting AUTO_DIRECT. Both first stop and verify standstill, then
switch the sole selected mode. The driver enforces exclusive setpoint routing
using fresh `vehicle/mode_state`; AUTO_DIRECT needs no extra opt-in setting.
The joystick node is `joy/game_controller_node` for standardized SDL ordering.
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
battery CAN receiver, joystick and manual
control, FlexiSoft bridge and safety monitor, Vanguard battery monitor and both
SICK scanner nodes. Autonomy and robot description/joint TF are not launched.
Wheel odometry does publish `odom -> base_link` on the namespaced `tf` topic.
The separate [mobotic_description](../mobotic_description/README.md) package now
provides an optional standalone description launch with confirmed CAD floor/axis
placement; ROS/RViz validation is still required before integration into bringup.

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

`mobotic_odometry` integrates measured `agv_vel` (derived from all four modules),
including sideways crab motion and yaw, and publishes `/moboterra/odometry`.
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
ros2 topic info /moboterra/odometry -v
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

Driver services are available under the robot namespace, e.g.
`/moboterra/clear_errors`. `enable_all_drives` requires a fresh enable lease.
`disable_all_drives` is not a persistent override while the supervisor requests enable.

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

## Inspect the system

```bash
ros2 node list
ros2 topic list
ros2 topic echo /moboterra/vehicle/mode_state
ros2 topic echo /moboterra/wheel_modules/status
ros2 topic echo /moboterra/diagnostics
rqt_graph
```

The [project graph](../../docs/moboterra_rqt_graph.md) is source-derived, not a
captured live graph. Missing publishers/subscribers may change what rqt displays.
