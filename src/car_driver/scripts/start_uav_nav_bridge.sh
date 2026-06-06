#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

source /opt/ros/humble/setup.bash
cd /home/elf/car/car_ws
source install/setup.bash

# Natural-driving Nav2 + EKF odom is the one-click default. To return to the
# stable raw-odom chain, also set use_ekf_odom:=false publish_odom_tf:=true.
exec ros2 launch car_driver uav_nav_bridge_bringup.launch.py "$@"
