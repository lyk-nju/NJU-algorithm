#!/bin/bash
# 远程查看导航可视化
export ROS_DOMAIN_ID=42

ros2 run rviz2 rviz2 -d /home/nvidia/NJU-algorithm/navigation_task/src/rm_nav_bringup/rviz/nav2.rviz
