#!/bin/bash

# Source ROS
source /mobotic_ws/devel/setup.bash

export ROBOT_NAME=mobotic
roslaunch mobotic_simulation four_wheel_robot.launch
