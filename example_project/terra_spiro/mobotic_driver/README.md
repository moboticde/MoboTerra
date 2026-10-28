# About

A ROS2 package that contains the driver for [Mobotic](https://www.mobotic.de/en/) products. The driver was tested
with [Mobotic ST-1000](https://www.mobotic.de/en/media/specifications/products/Mobotic_Steered-Traction_Unit_1c7xIUQ.pdf)
steered-traction drive.

This package implements the communication with the Mobotic ST-1000 drive using the CAN. The driver is implemented in C++
and uses the [SocketCAN](https://en.wikipedia.org/wiki/SocketCAN) interface to communicate with the CAN bus. The driver
is implemented as a ROS2 node and provides the following functionalities:

- Initializing the drive via CAN
- Reading the feedback from the drive
- Sending the velocity/steering/current commands to the drive
- Watchdog functionality (If no inputs are received from the driver for a certain period, the driver will stop the
  drive).

# How to build

## Pre-requisites

- Ubuntu 24.04
- ROS2 Jazzy

## Install ROS2

To install ROS2 follow the official instructions provided here.

Make sure to install the version that corresponds to the branch you require (`ros1` which is ROS1 Noetic or `main` which
is ROS2 Jazzy) or slight modifications may be required to
compile the code.

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

## Build the package

1. Create a Development Workspace
   `mkdir -p ~/colcon_ws/src`
2. Clone the `mobotic_driver` repository

```bash
cd ~/dev_ws/src
git clone https://gitlab.com/romb-technologies/mobotic.git
```

3. Build the workspace

```bash
cd ~/dev_ws
rosdep install -i --from-path src --rosdistro jazzy
colcon build
```

## Configure the driver parameters

The driver parameters are stored in the `mobotic_driver.yaml` file. The file is located in the `config` directory of the
repository. The file contains the following parameters:

- `can_interface` - The CAN interface to use for communication with the drive. (string)
- `feedback_rate` - The rate at which the driver reads the feedback from the drive (seconds) (double)
- `watchdog_timeout` - The timeout for the watchdog (seconds) (double)
- `diagnostic_sdo_period` - Period for operational status and error SDO reads (seconds, default `1.0`)
- `traction_pdo_enabled` - Use the traction RPDO/TPDO control loop (boolean, default `true`)
- `telemetry_pdo_enabled` - Configure PDO telemetry for steering position/velocity, current, temperature, and voltage
  (boolean, default `true`)
- `traction_pdo_period` - CANopen SYNC and traction-command period (seconds, default `0.02`, or 50 Hz)
- `telemetry_pdo_sync_divider` - Number of SYNC frames per telemetry sample (integer, default `1`). With the default
  50 Hz SYNC, a divider of 1 produces telemetry at 50 Hz.
- `traction_max_profile_velocity` - The EC controller's CANopen `0x607F` maximum profile velocity
  (controller increments/s) (integer)
- `drives.names` - The name of the drives (string). The name must match the `frame_id` of the drive joint in the
  `kinematics` node. (e.g. `front_left_steering`)
- `drives.id` - The CAN ID of the drives. The id is a hex string (e.g. `0x01`).
- `drives.modes` - The mode of the drives ('velocity', 'steering', 'current')
- `drives.resolutions` - The resolution of the drives (ticks) (integer)
- `drives.min_steering_velocities` - The minimum steering velocities of the drives (rpm) (integer)
- `drives.max_steering_velocities` - The maximum steering velocities of the drives (rpm) (integer)
- `drives.min_traction_velocities` - The minimum traction velocities of the EC drives (controller increments/s)
  (integer)
- `drives.max_traction_velocities` - The maximum traction velocities of the EC drives (controller increments/s)
  (integer)
- `drives.gear_ratios` - The gear ratios of the drives (double)
- `drives.min_currents` - The minimum currents of the drives (mA) (integer)
- `drives.max_currents` - The maximum currents of the drives (mA) (integer)

All `drives.*` lists must be of the same size or else the driver will not initialize.

## Run the driver

1. Launch the driver

```bash
ROBOT_NAME=mobotic ros2 launch mobotic_driver mobotic_driver.launch 
```

# Published Topics

- `can_tx` - The CAN messages that are sent to the drives (`can_msgs/frame`).
- `joint_states` - The feedback from the drives listed in the (`sensor_msgs/JointState`).

# Subscribed Topics

- `can_rx` - The CAN messages that are received from the drives (`can_msgs/frame`).
- `joint_setpoints` - The setpoints for the drives listed in the (`sensor_msgs/JointState`).

# Provided Services

| Service Name              | Service Type           | Purpose                               |
|:--------------------------|:-----------------------|:--------------------------------------|
| `~/initialize_all_drives` | `std_srvs/srv/Trigger` | Service to **initialize all drives**. |
| `~/enable_all_drives`     | `std_srvs/srv/Trigger` | Service to **enable all drives**.     |
| `~/disable_all_drives`    | `std_srvs/srv/Trigger` | Service to **disable all drives**.    |
| `~/clear_errors`          | `std_srvs/srv/Trigger` | Service to **clear all errors**.      |

# Specifics

1. The drivers are **initialized** when the _initialize_all_drives_ ROS service is called.
2. The watchdog will send the **stop command** to the drives if no inputs are received for a certain period. The timeout
   is defined by the _watchdog_timeout_ parameter.
3. The feedback is returned only after ALL feedbacks are received from ALL the drives.
4. The driver will **stop** and **disable** the drives if the driver is **terminated** (crashes don't disable the
   drives).
5. The driver will be reinitialized if the driver is **restarted**.
6. The driver converts **traction velocity**, **steering**, and **current** commands specified in SI derived units (rad,
   rad/s, A) to the drive's units (EC controller increments/s, ticks, steering RPM, and mA).
7. The driver converts **traction velocity**, **steering**, and **current** feedback from the drive's units (EC
   controller increments/s, ticks, steering RPM, and mA) to SI derived units (rad, rad/s, A).

## Telemetry PDO layout

The PDO mapping is applied when `initialize_all_drives` or `enable_all_drives` is requested.

| Drive | COB-ID | PDO payload |
|:------|:-------|:------------|
| Traction | `0x280 + node ID` | `0x6078:00` actual current (int16), `0x22A2:00` drive temperature (uint16) |
| Steering | `0x180 + node ID` | `0x3762:00` position (int32), `0x3A04:01` velocity (int32) |
| Steering | `0x280 + node ID` | `0x3111:00` bus voltage (uint32), `0x3262:01` actual current (int32) |
| Steering | `0x380 + node ID` | `0x3114:00` controller temperature (int32), `0x3262:00` RMS current (int32) |

The Elmo `0x6079:00` DC-link voltage and traction motor temperature `0x22A4:00` are read by SDO. The supplied EDS does
not list `0x22A4:00`, but the installed traction controllers accept expedited two-byte reads of that object.

The telemetry recorder defaults to 50 Hz JSON/CSV snapshots (`request_hz`) and 50 Hz PDO telemetry. Only traction
voltage uses the 10 Hz SDO path (`sdo_poll_hz`). Acceleration, deceleration, traction motor-sensor temperature, serial
number, and steering errors use the 1 Hz SDO path (`sdo_slow_poll_hz`); traction status/errors and steering status come
from PDOs or passively observed driver replies. Its per-node SDO state machines run concurrently, with at most one
outstanding request per node.
Traction RMS current is calculated over the latest 50 actual-current PDO samples (one second at 50 Hz). Steering RMS
current comes directly from `0x3262:00` in TPDO3. Traction actual current uses the configured fixed scale
(`traction_current_scale_ma_per_permille`, default 60 mA per permille), so no rated-current or RMS-current SDO is sent.

# Virtual Mobotic Drive

The `mobotic_driver` package contains a virtual Mobotic drive that can be used for testing purposes. The virtual drive
is implemented as a ROS2 node and provides the same functionalities as the real drive. The virtual drive is useful for
testing the driver without the need for the real hardware.

## Parameters

The C++ node is configured using the following internal parameter:

| Parameter Name | Type | Description                                                |
|:---------------|:-----|:-----------------------------------------------------------|
| `can_node_id`  | int  | The `can_node_id` of the virtual Mobotic drive to be used. |

## How to run the virtual drive(s)

1. Launch the virtual drive

```bash
ROBOT_NAME=mobotic ros2 launch mobotic_driver mobotic_drive_virtual.launch.py
```

The virtual drive(s) communicate using the following topics:

1. `can_rx` - The CAN messages that are received from the CAN bus (`can_msgs/frame`). The requests are sent to the
   virtual drive(s) via CAN using this topic.
2. `can_tx` - The CAN messages that are sent to the CAN bus (`can_msgs/frame`). The responses from the virtual drive(s)
   is sent via CAN using this topic.

When the virtual drive(s) receive a request for feedback, they will respond with the latest setpoints they received
(**perfect feedback**). The responses to requests are hard-coded, there is **no state** in the virtual drive(s).
The `current`, `velocity`, and `steering` setpoints and the `mode` is remembered by the virtual drive(s) and used to
send a proper response when a READ request is received.
