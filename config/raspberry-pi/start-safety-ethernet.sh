#!/usr/bin/env bash
set -eo pipefail
source /home/mobotic/Downloads/MoboTerra-main/install/setup.bash
export ROS_DOMAIN_ID=42
exec ros2 launch /home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/safety-ethernet.launch.py
