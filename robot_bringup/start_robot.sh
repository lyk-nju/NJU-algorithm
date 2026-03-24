#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

AUTO_AIMER_CMD="cd ${ROOT_DIR}/vision_task/armor_task/build && ./auto_aimer_test"
BRINGUP_CMD="source /opt/ros/humble/setup.bash && \
source ${ROOT_DIR}/navigation_task/install/setup.bash && \
source ${ROOT_DIR}/vision_task/hiki_ros2/install/setup.bash && \
source ${ROOT_DIR}/decision_task/install/setup.sh && \
source ${ROOT_DIR}/robot_bringup/install/setup.bash && \
ros2 launch robot_bringup robot_bringup.launch.py"

if command -v gnome-terminal >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
	gnome-terminal --title="Auto Aimer" -- bash -lc "${AUTO_AIMER_CMD}; exec bash"
	sleep 3
	gnome-terminal --title="Robot Bringup" -- bash -lc "${BRINGUP_CMD}; exec bash"
else
	echo "未检测到可用图形终端（gnome-terminal/DISPLAY），退化为当前 shell 启动。"
	bash -lc "${AUTO_AIMER_CMD}" &
	sleep 3
	bash -lc "${BRINGUP_CMD}"
fi
