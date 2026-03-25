#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/nvidia/NJU-algorithm/vision_task/hiki_ros2/install/setup.bash

ros2 launch hik_camera hik_camera.launch.py publish_camera_info:=false enable_timing_log:=true 


