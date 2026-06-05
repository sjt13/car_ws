#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

source /opt/ros/humble/setup.bash

if [[ -f /home/ros2/car/car_ws/install/setup.bash ]]; then
  source /home/ros2/car/car_ws/install/setup.bash
fi

exec rviz2 -d /home/ros2/car/car_ws/src/car_description/rviz/car.rviz
