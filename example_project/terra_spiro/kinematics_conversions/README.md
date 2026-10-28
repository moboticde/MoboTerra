# kinematics_conversions

A ROS 2 package for mobile robot kinematics conversions. This package provides nodes to convert between `sensor_msgs/JointState`, `geometry_msgs/TwistStamped`, and `ackermann_msgs/AckermannDriveStamped` messages.

## Nodes

* **`joint_state_to_twist`**: A C++ node that calculates the robot's velocity (`TwistStamped`) from individual wheel velocities and steering angles (`JointState`).
* **`twist_to_acker`**: A C++ node that converts a `TwistStamped` command into an `AckermannDriveStamped` command.
* **`joint_states_to_acker.py`**: A Python node that provides two-way conversion between `JointState` messages and `AckermannDriveStamped` messages.

## Installation

### Prerequisites
* ROS 2 Jazzy Jubilee
* Eigen3 (`sudo apt-get install libeigen3-dev`)
* `colcon` build tools

### Building from Source

1.  Clone this repository into your ROS 2 workspace's `src` directory:
    ```bash
    cd ~/ros2_ws/src
    git clone <your-repo-url>
    ```
2.  Build the package using `colcon`:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select kinematics_conversions
    ```
3.  Source your workspace:
    ```bash
    source install/setup.bash
    ```

## Usage

You can run each conversion node using its respective launch file.

### JointState to Ackermann

```bash
ros2 launch kinematics_conversions joint_state_to_acker.launch.py
```

### JointState to Twist

```bash
ros2 launch kinematics_conversions joint_state_to_twist.launch.py
```


### Twist to Ackermann

```bash
ros2 launch kinematics_conversions twist_to_acker.launch.py wheelbase:=0.4
```
