#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ${SCRIPT_DIR}/navigation_task/install/setup.bash
source ${SCRIPT_DIR}/vision_task/hiki_ros2/install/setup.bash
source ${SCRIPT_DIR}/decision_task/install/setup.sh
source ${SCRIPT_DIR}/robot_bringup/install/setup.bash

ros2 launch robot_bringup robot_bringup.launch.py
