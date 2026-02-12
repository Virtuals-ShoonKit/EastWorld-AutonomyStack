#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/nvidia/Desktop/EastWorld-AutonomyStack/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/Desktop/EastWorld-AutonomyStack/src/eastworld_bringup/config/cyclonedds.xml

exec ros2 launch eastworld_bringup bringup.launch.py "$@"
