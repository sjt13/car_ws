#!/usr/bin/env bash
set -u

echo "[kill_ros] stopping ROS2 launches and nodes for user: ${USER}"

PATTERNS=(
  "ros2 launch"
  "launch.launch"
  "launch_ros"
  "static_transform_publisher"
  "robot_state_publisher"
  "joint_state_publisher"
  "rviz2"
  "nav2_"
  "lifecycle_manager"
  "map_server"
  "amcl"
  "controller_server"
  "planner_server"
  "smoother_server"
  "behavior_server"
  "bt_navigator"
  "waypoint_follower"
  "velocity_smoother"
  "base_driver_node"
  "rplidar_node"
  "ekf_node"
  "slam_toolbox"
  "map_odom_tf_keepalive_node"
  "goal_slam_navigator_node"
  "openclaw_goal_decision_node"
  "orbbec_camera"
  "target_mapper_node"
  "uav_target_bridge_node"
  "yolo"
)

kill_matches() {
  local signal="$1"
  local killed=0

  for pattern in "${PATTERNS[@]}"; do
    local pids
    pids="$(pgrep -u "${USER}" -f "${pattern}" | grep -v "^$$$" || true)"
    if [[ -n "${pids}" ]]; then
      echo "[kill_ros] ${signal} ${pattern}: ${pids//$'\n'/ }"
      kill "-${signal}" ${pids} 2>/dev/null || true
      killed=1
    fi
  done

  return "${killed}"
}

kill_matches TERM || true
sleep 2
kill_matches KILL || true

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "[kill_ros] remaining matched processes:"
remaining=0
for pattern in "${PATTERNS[@]}"; do
  if pgrep -a -u "${USER}" -f "${pattern}" >/dev/null; then
    pgrep -a -u "${USER}" -f "${pattern}" || true
    remaining=1
  fi
done

if [[ "${remaining}" -eq 0 ]]; then
  echo "[kill_ros] done, no matched ROS2 launch/node processes remain."
else
  echo "[kill_ros] some matched processes are still present; check manually before restarting."
fi
