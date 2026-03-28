#!/bin/bash
set -o pipefail

STATE_FILE="${XDG_RUNTIME_DIR:-/tmp}/robot_nav_healthcheck.state"
GRACE_PERIOD_SEC=120
MAX_CONSECUTIVE_FAILURES=3

# 与 robot_nav.service 保持一致，避免 healthcheck 误入错误的 ROS Domain。
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 仅在robot_nav.service处于active时做检查
if ! systemctl --user is-active --quiet robot_nav.service; then
  rm -f "$STATE_FILE"
  exit 0
fi

source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/navigation_task/install/setup.bash >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/vision_task/hiki_ros2/install/setup.bash >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/decision_task/install/setup.sh >/dev/null 2>&1 || true
source /home/nvidia/NJU-algorithm/robot_bringup/install/setup.bash >/dev/null 2>&1 || true

# 启动初期 Nav2/定位链路尚未完全 ready，给足宽限时间，避免冷启动误判。
service_start_us=$(systemctl --user show robot_nav.service -p ActiveEnterTimestampMonotonic --value 2>/dev/null)
uptime_sec=$(awk '{print int($1)}' /proc/uptime 2>/dev/null)

if [[ -n "$service_start_us" && -n "$uptime_sec" && "$service_start_us" =~ ^[0-9]+$ ]]; then
  now_us=$((uptime_sec * 1000000))
  service_age_sec=$(((now_us - service_start_us) / 1000000))
  if (( service_age_sec < GRACE_PERIOD_SEC )); then
    rm -f "$STATE_FILE"
    exit 0
  fi
fi

# 1) Nav2容器进程是否还在
has_nav2_container=0
if pgrep -fa "component_container_mt.*__node:=nav2_container" >/dev/null 2>&1; then
  has_nav2_container=1
fi

# 2) 导航 Action 是否注册成功。
# 比起直接看 goal topic 的订阅者数量，action list 对启动中瞬态更不敏感。
action_list=$(timeout 8s ros2 action list 2>/dev/null || true)
has_navigate_to_pose=0
has_navigate_through_poses=0

if printf '%s\n' "$action_list" | grep -Fx "/navigate_to_pose" >/dev/null 2>&1; then
  has_navigate_to_pose=1
fi

if printf '%s\n' "$action_list" | grep -Fx "/navigate_through_poses" >/dev/null 2>&1; then
  has_navigate_through_poses=1
fi

if [[ "$has_navigate_to_pose" -eq 1 || "$has_navigate_through_poses" -eq 1 ]]; then
  nav_status_ok=1
else
  nav_status_ok=0
fi

if [[ "$has_nav2_container" -eq 1 && "$nav_status_ok" -eq 1 ]]; then
  rm -f "$STATE_FILE"
  exit 0
fi

fail_count=0
if [[ -f "$STATE_FILE" ]]; then
  fail_count=$(cat "$STATE_FILE" 2>/dev/null)
fi

if ! [[ "$fail_count" =~ ^[0-9]+$ ]]; then
  fail_count=0
fi

fail_count=$((fail_count + 1))
printf '%s\n' "$fail_count" > "$STATE_FILE"

echo "[healthcheck] nav unhealthy ($fail_count/$MAX_CONSECUTIVE_FAILURES): nav2_container=$has_nav2_container, navigate_to_pose=$has_navigate_to_pose, navigate_through_poses=$has_navigate_through_poses"

# 连续多次失败才重启，避免偶发 discovery 抖动导致误重启。
if (( fail_count >= MAX_CONSECUTIVE_FAILURES )); then
  rm -f "$STATE_FILE"
  echo "[healthcheck] restarting robot_nav.service after consecutive failures"
  if ! systemctl --user restart robot_nav.service; then
    echo "[healthcheck] WARN: restart robot_nav.service failed"
  fi
fi

exit 0
