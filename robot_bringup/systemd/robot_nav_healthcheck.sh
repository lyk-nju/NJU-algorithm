#!/bin/bash
set -o pipefail

# 仅在robot_nav.service处于active时做检查
if ! systemctl --user is-active --quiet robot_nav.service; then
  exit 0
fi

source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/navigation_task/install/setup.bash >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/vision_task/hiki_ros2/install/setup.bash >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/decision_task/install/setup.sh >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/robot_bringup/install/setup.bash >/dev/null 2>&1 || true

# 1) Nav2容器进程是否还在
has_nav2_container=0
if pgrep -fa "component_container_mt.*__node:=nav2_container" >/dev/null 2>&1; then
  has_nav2_container=1
fi

# 2) 导航Action是否就绪（检查goal话题的订阅者，有订阅者说明Nav2已注册action服务）
nav_goal_sub_through=$(ros2 topic info /navigate_through_poses/_action/goal 2>/dev/null | awk -F': ' '/Subscriber count/{print $2; exit}')
nav_goal_sub_to=$(ros2 topic info /navigate_to_pose/_action/goal 2>/dev/null | awk -F': ' '/Subscriber count/{print $2; exit}')
nav_goal_sub_through=${nav_goal_sub_through:-0}
nav_goal_sub_to=${nav_goal_sub_to:-0}

if [[ "$nav_goal_sub_through" -gt 0 || "$nav_goal_sub_to" -gt 0 ]]; then
  nav_status_ok=1
else
  nav_status_ok=0
fi

# 任一关键条件不满足，重启导航服务
if [[ "$has_nav2_container" -eq 0 || "$nav_status_ok" -eq 0 ]]; then
  echo "[healthcheck] nav unhealthy: nav2_container=$has_nav2_container, through_subs=$nav_goal_sub_through, to_subs=$nav_goal_sub_to -> restarting robot_nav.service"
  if ! systemctl --user restart robot_nav.service; then
    echo "[healthcheck] WARN: restart robot_nav.service failed"
  fi
fi

exit 0
