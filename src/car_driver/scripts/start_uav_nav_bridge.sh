#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

source /opt/ros/humble/setup.bash
cd /home/elf/car/car_ws
source install/setup.bash

exec ros2 launch car_driver uav_nav_bridge_bringup.launch.py "$@"
