#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

source /opt/ros/humble/setup.bash
cd /home/elf/car/car_ws
source install/setup.bash

stale_patterns=(
  '/opt/ros/humble/bin/ros2 launch car_driver goal_slam_nav_bringup.launch.py'
  '/home/elf/car/car_ws/install/car_driver/lib/car_driver/base_driver_node'
  '/home/elf/car/car_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node'
  '/opt/ros/humble/lib/robot_localization/ekf_node'
  '/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node'
  'map_odom_tf_keepalive_node'
  '/opt/ros/humble/lib/nav2_controller/controller_server'
  '/opt/ros/humble/lib/nav2_smoother/smoother_server'
  '/opt/ros/humble/lib/nav2_planner/planner_server'
  '/opt/ros/humble/lib/nav2_behaviors/behavior_server'
  '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator'
  '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower'
  '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother'
  '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager'
  '/opt/ros/humble/lib/tf2_ros/static_transform_publisher'
  '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
  '/home/elf/car/car_ws/install/car_yolo/lib/car_yolo/uav_target_bridge_node'
  '/home/elf/car/car_ws/install/car_yolo/lib/car_yolo/openclaw_goal_decision_node'
  '/home/elf/car/car_ws/install/car_driver/lib/car_driver/goal_slam_navigator_node.py'
)

for pattern in "${stale_patterns[@]}"; do
  stale_pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  [[ -z "${stale_pids}" ]] || kill -TERM ${stale_pids} 2>/dev/null || true
done

for _ in $(seq 1 30); do
  stale_found=false
  for pattern in "${stale_patterns[@]}"; do
    if pgrep -f "${pattern}" >/dev/null 2>&1; then
      stale_found=true
      break
    fi
  done
  [[ "${stale_found}" == false ]] && break
  sleep 0.1
done

for pattern in "${stale_patterns[@]}"; do
  stale_pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  [[ -z "${stale_pids}" ]] || kill -KILL ${stale_pids} 2>/dev/null || true
done

exec ros2 launch car_driver goal_slam_nav_bringup.launch.py "$@"
