#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <map_server|amcl|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|all>"
  exit 1
fi

activate_node() {
  local node="$1"
  local state
  state=$(ros2 lifecycle get "/${node}" 2>/dev/null || true)

  if echo "$state" | grep -qi 'active'; then
    echo "==> ${node} already active"
    return 0
  fi

  if echo "$state" | grep -qi 'unconfigured'; then
    echo "==> configuring ${node}"
    ros2 lifecycle set "/${node}" configure || true
    state=$(ros2 lifecycle get "/${node}" 2>/dev/null || true)
  fi

  if echo "$state" | grep -qi 'inactive'; then
    echo "==> activating ${node}"
    ros2 lifecycle set "/${node}" activate || true
    state=$(ros2 lifecycle get "/${node}" 2>/dev/null || true)
  fi

  if echo "$state" | grep -qi 'active'; then
    echo "==> ${node} active"
    return 0
  fi

  echo "==> fallback transition probing for ${node}"
  ros2 lifecycle set "/${node}" configure || true
  ros2 lifecycle set "/${node}" activate || true
  state=$(ros2 lifecycle get "/${node}" 2>/dev/null || true)

  if echo "$state" | grep -qi 'active'; then
    echo "==> ${node} active"
    return 0
  fi

  echo "==> ${node} current state after retries:"
  echo "$state"
  echo "Cannot auto-activate ${node}."
  exit 3
}

case "$1" in
  map_server|amcl|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother)
    activate_node "$1"
    ;;
  all)
    activate_node map_server
    activate_node amcl
    activate_node controller_server
    activate_node planner_server
    activate_node behavior_server
    activate_node bt_navigator
    activate_node waypoint_follower
    activate_node velocity_smoother
    ;;
  *)
    echo "Unknown target: $1"
    exit 2
    ;;
esac
