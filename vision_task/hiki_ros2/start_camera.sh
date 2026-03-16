#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch hik_camera hik_camera.launch.py


##you can visualize the camera image in riviz2
##start another terminal and start rviz2
##click "add" and select "by topic"
##choose "image_raw"
