# About

This ROS 2 package contains the **Vehicle Supervisor** node. The vehicle supervisor initializes the vehicle, monitors
the state of the safety subsystem and supervises the velocity of the vehicle based on the current safety state.

The node is implemented in C++ and provides the following core functionalities:

- **Velocity Command Filtering:** It subscribes to user command velocities (`cmd_vel`) and applies dynamic limits based
  on configured maximums, acceleration limits, and safety status.
- **Safety Velocity Implementation:** It limits the vehicle's maximum velocity based on safety feedback (e.g., from a
  SICK scanner).
- **Vehicle initialization:** It provides a service to enable/disable the drives and acts as a client to trigger
  initialization
  and high-voltage (HV) enabling routines in other nodes.

-----

## Driver Parameters

The `vehicle_supervisor` node is configured using the following launch arguments, which correspond directly to its
internal parameters:

| Launch Argument                 | Parameter Name                  | Type   | Default Value | Description                                                              |
|:--------------------------------|:--------------------------------|:-------|:--------------|:-------------------------------------------------------------------------|
| `max_velocity_safety_disabled`  | `max_velocity_safety_disabled`  | double | `0.25`        | **Max Linear Velocity** allowed when the safety sub-safety is disabled.  |
| `max_velocity_warning_breached` | `max_velocity_warning_breached` | double | `0.30`        | **Velocity Threshold** when the warning zone of SICK is breached.        |
| `max_acceleration`              | `max_acceleration`              | double | `0.5`         | **Maximum Linear Acceleration** limit (m/s²).                            |
| `max_acceleration_angular`      | `max_acceleration_angular`      | double | `1.5`         | **Overall Maximum Angular Acceleration** limit (rad/s²) for the vehicle. |
| `max_velocity`                  | `max_velocity`                  | double | `1.25`        | **Overall Maximum Linear Velocity** limit (m/s) for the vehicle.         |
| `wheelbase`                     | `wheelbase`                     | double | `1.65`        | **Distance** from the front to the back wheels (m).                      |

-----

## Run the Supervisor

The node is launched using its dedicated launch file. Parameters can be overridden directly from the command line.

1. Launch the supervisor node:

```bash
ros2 launch terraspiro_vehicle_supervisor terraspiro_vehicle_supervisor.launch.py 
```

-----

## Topics and Services

The node interacts with the rest of the ROS 2 system via the following topics and services:

### Subscribed Topics

| Topic Name                    | Message Type                             | Purpose                                                                                        |
|:------------------------------|:-----------------------------------------|:-----------------------------------------------------------------------------------------------|
| `cmd_vel` (or similar)        | `geometry_msgs/msg/TwistStamped`         | **Desired velocity commands** from a user or planner.                                          |
| `agv_vel` (or similar)        | `geometry_msgs/msg/TwistStamped`         | **Actual vehicle velocity feedback** from the drive system.                                    |
| `battery_status` (or similar) | `sensor_msgs/msg/BatteryState`           | **Current battery status** (e.g., voltage, charge, health).                                    |
| `sick_info` (or similar)      | `terraspiro_sick_interface/msg/SickInfo` | **Safety state/info** from the SICK laser scanner (e.g., free/warning/protected field status). |

### Published Topics

| Topic Name                        | Message Type                     | Purpose                                                                    |
|:----------------------------------|:---------------------------------|:---------------------------------------------------------------------------|
| `cmd_vel_supervised` (or similar) | `geometry_msgs/msg/TwistStamped` | **Filtered and limited velocity commands** sent to the drive control node. |

### Provided Services

| Service Name      | Service Type           | Purpose                                              |
|:------------------|:-----------------------|:-----------------------------------------------------|
| `~/enable_drives` | `std_srvs/srv/Trigger` | Service to **enable or disable the vehicle drives**. |

### Used Service Clients

The supervisor acts as a client to trigger essential actions in other nodes (likely drive and battery managers):

| Client Name             | Service Type           | Purpose                                                                         |
|:------------------------|:-----------------------|:--------------------------------------------------------------------------------|
| `enable_high_voltage`   | `std_srvs/srv/Trigger` | Client to request the **enabling of high voltage** from the battery driver.     |
| `enable_all_drives`     | `std_srvs/srv/Trigger` | Client to request the **enabling of the drives** from the Mobotic driver.       |
| `initialize_all_drives` | `std_srvs/srv/Trigger` | Client to request the **initialization** of the drives from the Mobotic driver. |