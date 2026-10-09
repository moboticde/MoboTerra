# MoboTerra ROS graph

Source-derived hardware node/topic graph for the default `/moboterra` namespace,
updated 2026-10-08. This is **not a captured live rqt_graph** or proof of runtime
integration. Rectangles are nodes, rounded shapes are topics; dashed external
nodes/connections are optional consumers/producers not launched by bringup.
Hardware arrows denote transport, not ROS topics.

All topic labels below are namespace-relative. Generic ROS `rosout` and
`parameter_events` plumbing is omitted. Services are listed separately.

## Configuration ownership

`mobotic_config/config/platform.yaml` feeds driver, supervisor, kinematics,
manual caps, battery/safety mapping, scanner endpoints, virtual actuators and the
Xacro description. Adjacent YAML files own only their node policies. Configuration
loading is not a ROS node/topic and therefore does not add arrows to this graph.
Diagnostics use effective supervisor/driver freshness limits; scanner and odometry
health deadlines have their own diagnostic policy. Hardware topic routing below
is unchanged by this consolidation. See [configuration guide](../src/mobotic_config/README.md).

## Hardware profile: whole project

```mermaid
flowchart LR
  classDef rosnode fill:#6d28d91f,stroke:#6d28d9,stroke-width:1.5px
  classDef topic fill:#2563eb17,stroke:#2563eb,stroke-width:1px
  classDef hardware fill:#ea580c17,stroke:#ea580c,stroke-width:1.5px
  classDef external fill:#64748b12,stroke:#64748b,stroke-dasharray:5 4

  subgraph HW["Hardware / transport"]
    CANBUS[["SocketCAN can0: drives"]]:::hardware
    BATTERYBUS[["SocketCAN can1: Vanguard, listen-only"]]:::hardware
    FLEXI[["FlexiSoft TCP :9100"]]:::hardware
    GAMEPAD[["Logitech F310/F710 USB (temporary)"]]:::hardware
    FRONT_HW[["Front scanner 10.60.20.52"]]:::hardware
    REAR_HW[["Rear scanner 10.60.20.53"]]:::hardware
  end

  subgraph NS["Namespace /moboterra"]
    CANRX["socket_can_receiver"]:::rosnode
    CANTX["socket_can_sender"]:::rosnode
    BATTERYRX["battery_can/socket_can_receiver"]:::rosnode
    DRIVER["mobotic_driver"]:::rosnode
    BATTERY["vanguard_battery_monitor"]:::rosnode
    FLEXIBRIDGE["flexisoft_tcp_bridge"]:::rosnode
    SAFETY["safety_monitor"]:::rosnode
    JOY["joystick (game_controller_node)"]:::rosnode
    MANUAL["mobotic_manual_control (teleop)"]:::rosnode
    SUPERVISOR["mobotic_supervisor"]:::rosnode
    KINEMATICS["mobotic_kinematics"]:::rosnode
    ODOMETRY["mobotic_odometry"]:::rosnode
    FRONT_SCAN["front_left_scanner"]:::rosnode
    REAR_SCAN["rear_right_scanner"]:::rosnode
    PLATFORM_DIAG["platform_diagnostics"]:::rosnode

    T_CAN_RX(["can_rx"]):::topic
    T_CAN_TX(["can_tx"]):::topic
    T_BATTERY_CAN_RX(["battery_can/can_rx"]):::topic
    T_JOINTS(["joint_states"]):::topic
    T_MODULE_STATUS(["wheel_modules/status"]):::topic
    T_MODULE_COMMAND(["wheel_modules/command"]):::topic
    T_KIN_JOINT_COMMAND(["kinematics/joint_setpoints"]):::topic
    T_SUP_JOINT_COMMAND(["supervisor/joint_setpoints"]):::topic
    T_JOY(["joy"]):::topic
    T_MANUAL_CMD(["manual/cmd_vel"]):::topic
    T_MANUAL_STATE(["manual/state"]):::topic
    T_FLEXI(["flexisoft/status_byte"]):::topic
    T_FLEXI_RAW(["flexisoft/telegram"]):::topic
    T_SAFETY_IO(["safety/io_state"]):::topic
    T_SAFETY(["safety/state"]):::topic
    T_BATTERY_STATE(["battery/state"]):::topic
    T_BATTERY_SYSTEM(["battery/system_state"]):::topic
    T_CMD_VEL(["cmd_vel"]):::topic
    T_AGV_VEL(["agv_vel"]):::topic
    T_ODOMETRY(["odometry"]):::topic
    T_ODOM_TF(["tf: odom -> base_link (optional)"]):::topic
    T_MODE_STATE(["vehicle/mode_state"]):::topic
    T_MODE_REQUEST(["vehicle/mode_request"]):::topic
    T_AUTO_VEL(["autonomy/cmd_vel"]):::topic
    T_AUTO_DIRECT(["autonomy/joint_setpoints"]):::topic
    T_FRONT_SCAN(["scanner/front_left/scan"]):::topic
    T_FRONT_RAW(["scanner/front_left/raw_data"]):::topic
    T_FRONT_PATHS(["scanner/front_left/output_paths"]):::topic
    T_FRONT_EXT(["scanner/front_left/extended_laser_scan"]):::topic
    T_REAR_SCAN(["scanner/rear_right/scan"]):::topic
    T_REAR_RAW(["scanner/rear_right/raw_data"]):::topic
    T_REAR_PATHS(["scanner/rear_right/output_paths"]):::topic
    T_REAR_EXT(["scanner/rear_right/extended_laser_scan"]):::topic
    T_DIAG(["diagnostics"]):::topic
  end

  AUTONOMY["External autonomy (not implemented)"]:::external
  OPERATOR["External operator / commissioning client"]:::external
  MONITOR["External monitoring UI"]:::external

  CANBUS --> CANRX --> T_CAN_RX
  T_CAN_RX --> DRIVER
  BATTERYBUS --> BATTERYRX --> T_BATTERY_CAN_RX --> BATTERY
  DRIVER --> T_CAN_TX --> CANTX --> CANBUS

  FLEXI --> FLEXIBRIDGE --> T_FLEXI --> SAFETY
  FLEXIBRIDGE --> T_FLEXI_RAW
  SAFETY --> T_SAFETY --> SUPERVISOR
  SAFETY --> T_SAFETY_IO --> SUPERVISOR
  T_SAFETY --> PLATFORM_DIAG
  T_SAFETY_IO --> PLATFORM_DIAG

  GAMEPAD --> JOY --> T_JOY --> MANUAL
  MANUAL --> T_MANUAL_CMD --> SUPERVISOR
  MANUAL --> T_MANUAL_STATE --> SUPERVISOR
  MANUAL --> T_MODE_REQUEST
  T_MANUAL_STATE --> PLATFORM_DIAG

  BATTERY --> T_BATTERY_STATE
  T_BATTERY_STATE -.-> MONITOR
  BATTERY --> T_BATTERY_SYSTEM --> SUPERVISOR
  T_BATTERY_SYSTEM --> PLATFORM_DIAG

  AUTONOMY -.-> T_AUTO_VEL --> SUPERVISOR
  AUTONOMY -.-> T_AUTO_DIRECT --> SUPERVISOR
  AUTONOMY -.-> T_MODE_REQUEST
  OPERATOR -.-> T_MODE_REQUEST --> SUPERVISOR
  SUPERVISOR --> T_MODE_STATE
  T_MODE_STATE --> DRIVER
  T_MODE_STATE --> MANUAL
  T_MODE_STATE --> PLATFORM_DIAG
  T_MODE_STATE -.-> OPERATOR
  T_MODE_STATE -.-> AUTONOMY

  SUPERVISOR --> T_CMD_VEL --> KINEMATICS
  KINEMATICS --> T_KIN_JOINT_COMMAND --> DRIVER
  SUPERVISOR --> T_SUP_JOINT_COMMAND --> DRIVER
  SUPERVISOR --> T_MODULE_COMMAND --> DRIVER
  DRIVER --> T_MODULE_STATUS --> SUPERVISOR
  DRIVER --> T_JOINTS --> KINEMATICS
  T_JOINTS --> SUPERVISOR
  T_JOINTS -. read-only feedback .-> AUTONOMY
  KINEMATICS --> T_AGV_VEL --> SUPERVISOR
  T_AGV_VEL --> ODOMETRY --> T_ODOMETRY --> PLATFORM_DIAG
  T_ODOMETRY -.-> AUTONOMY
  ODOMETRY --> T_ODOM_TF
  ODOMETRY --> T_DIAG

  FRONT_HW --> FRONT_SCAN
  REAR_HW --> REAR_SCAN
  FRONT_SCAN --> T_FRONT_SCAN --> PLATFORM_DIAG
  FRONT_SCAN --> T_FRONT_RAW
  FRONT_SCAN --> T_FRONT_PATHS
  FRONT_SCAN --> T_FRONT_EXT
  REAR_SCAN --> T_REAR_SCAN --> PLATFORM_DIAG
  REAR_SCAN --> T_REAR_RAW
  REAR_SCAN --> T_REAR_PATHS
  REAR_SCAN --> T_REAR_EXT
  T_FRONT_SCAN -.-> AUTONOMY
  T_REAR_SCAN -.-> AUTONOMY

  DRIVER --> T_DIAG
  PLATFORM_DIAG --> T_DIAG
  FRONT_SCAN --> T_DIAG
  REAR_SCAN --> T_DIAG
  T_DIAG -.-> MONITOR
```

SocketCAN node labels are the expected names from the external bridge launch;
check the installed package's actual node names in the live graph. Scanner
auxiliary topics are the configured native-driver remappings; their runtime
availability depends on the scanner configuration/driver.

## Important distinctions

| Interface | Function |
|:--|:--|
| `manual/cmd_vel`, `autonomy/cmd_vel` | Candidate body velocities; only the supervisor selects one |
| `autonomy/joint_setpoints -> supervisor/joint_setpoints` | Optional external-kinematics motion path through supervisor; not wheel management |
| `cmd_vel -> kinematics/joint_setpoints` | Supervised velocity-to-joint motion path |
| `wheel_modules/command` | One `WheelModuleCommand`: enable/disable and clear errors on all units |
| `wheel_modules/status` | `WheelModuleStatusArray`: module readiness/faults with `feedback_fresh` |
| `vehicle/mode_request` | MANUAL/AUTO_VELOCITY requests only; Back toggles using reported mode; supervisor stops before switching |
| `vehicle/mode_state` | Authoritative mode for the USB Back toggle and driver-exclusive kinematics (0/1) or direct (2) routing |
| `joint_states -> agv_vel` | Measured joints/body velocity for standstill verification and odometry input |
| `agv_vel -> odometry` | Timestamped planar wheel pose/twist/covariance, not commanded-motion integration |
| `tf: odom -> base_link` | Optional wheel-odometry transform; disable when an external estimator owns it |
| `battery/state` | Single standard battery telemetry output; supervisor does not subscribe |
| `battery/system_state` | Aggregate readiness/minimum SOC; supervisor and platform diagnostics subscribe |
| `safety/io_state` | Additional supervisor interlock input and diagnostics; unknown STO/enable mappings remain explicit |
| `diagnostics` | Shared driver/platform/native-scanner health output; no control authority |

The teleop role is implemented by `mobotic_manual_control`; `joystick` remains a
separate `game_controller_node`. The supervisor receives manual state rather than subscribing
directly to raw `joy`.

Scanner protective fields reach ROS supervision through FlexiSoft, not through a
new scan-based safety algorithm. Laser scans feed diagnostics and optional autonomy.
Physical ELMO/FlexiSoft STO wiring is **not** a ROS edge. Raw ELMO safety-register
values appear inside driver diagnostics; their bit meanings are not guessed.

Enable depends on fresh ready safety/battery and SOC >=20%. Motion additionally
requires a valid selected source and fresh enabled fault-free drive feedback.
A fresh status publication alone is insufficient when `feedback_fresh=false`.
Default bringup now includes wheel odometry and odom-to-base TF, but still not
`robot_state_publisher`. The optional
standalone [description launch](../src/mobotic_description/README.md) adds this
branch, without changing motion routing (CAD floor/axis placement is now confirmed):

```mermaid
flowchart LR
  D["mobotic_driver"] --> JS(["joint_states"])
  JS --> RSP["robot_state_publisher (optional)"]
  URDF["mobotic_description URDF"] --> RSP
  RSP --> TF(["tf: moving joints"])
  RSP --> STATIC(["tf_static: fixed joints"])
  RSP --> DESC(["robot_description"])
```

These topics use the same `/moboterra` namespace as odometry TF. The description
does not duplicate `odom -> base_link`. Its `tf_static` now includes nominal
CAD/manufacturer-aligned `chassis -> front_left_scan` and
`chassis -> rear_right_scan` transforms matching the scanner message frame IDs;
these are not physically calibrated extrinsics.

## Services (not displayed by rqt_graph)

| Default service | Type | Server |
|:--|:--|:--|
| `/moboterra/vehicle/set_mode` | `mobotic_interfaces/srv/SetVehicleMode` | Supervisor; modes 0/1/2, only API accepting AUTO_DIRECT |
| `/moboterra/initialize_all_drives` | `std_srvs/srv/Trigger` | Driver |
| `/moboterra/enable_all_drives` | `std_srvs/srv/Trigger` | Driver; fresh enable lease required |
| `/moboterra/disable_all_drives` | `std_srvs/srv/Trigger` | Driver |
| `/moboterra/clear_errors` | `std_srvs/srv/Trigger` | Driver |
| `/moboterra/odometry/reset` | `std_srvs/srv/Trigger` | Odometry; fresh measured standstill required |

See the [supervisor interface contract](supervisor_interfaces.md) and
[watchdog/commissioning guide](hardware_monitoring.md).

## Virtual profile differences

The common driver/kinematics/odometry/supervisor/diagnostics interfaces remain; these
local nodes replace the physical CAN and safety/battery boundaries:

```mermaid
flowchart LR
  D["mobotic_driver"] --> TX(["can_tx"])
  TX --> V["Four MiControl steering + four ELMO traction simulators"]
  V --> RX(["can_rx"]) --> D
  M["mock_platform_state"] --> S(["safety/state"])
  M --> IO(["safety/io_state"])
  M --> B(["battery/system_state"])
  S --> SUP["mobotic_supervisor"]
  IO --> SUP
  B --> SUP
  S --> PD["platform_diagnostics"]
  IO --> PD
  B --> PD
```

No SocketCAN bridge, physical scanners, FlexiSoft bridge/monitor or Vanguard
monitor runs in virtual mode. Manual nodes are still optional. Driver PDO paths
use the same configuration as hardware mode. Virtual nodes implement the production
SDO/enable and synchronous PDO layouts, not physical motor/brake/STO behavior.
Protocol regression tests pass; full ROS end-to-end simulation is not yet validated.
Mock readiness does not bypass drive-feedback gates, and no mock public
`battery/state` is published.

## Inspect the live graph

On the configured ROS host, after building/sourcing the workspace:

```bash
ros2 launch mobotic_bringup moboterra.launch.py stack_type:=hardware robot_name:=moboterra
rqt_graph
```

Select **Nodes/Topics (all)** and disable **Hide leaf topics**.
External autonomy inputs/consumers appear only when matching nodes are running;
some unpublished/unconnected topics may still be hidden by rqt. Use
`ros2 topic list -t`, `ros2 node info /moboterra/mobotic_supervisor` and
`ros2 service list -t` to inspect interfaces not visible in the graph.
