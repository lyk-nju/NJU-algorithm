#!/bin/bash

echo "Killing NJU-algorithm project processes..."

# Kill ROS2 launch shell processes
pkill -9 -f "robot_bringup.launch.py"

# Kill processes from navigation_task, vision_task, decision_task directories
pkill -9 -f "navigation_task"
pkill -9 -f "vision_task"
pkill -9 -f "decision_task"
pkill -9 -f "robot_bringup"

# Kill specific ROS2 nodes
pkill -9 -f "fast_lio"
pkill -9 -f "livox_ros_driver2"
pkill -9 -f "livox_lidar"
pkill -9 -f "amcl"
pkill -9 -f "nav2"
pkill -9 -f "map_server"
pkill -9 -f "lifecycle_manager"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "pointcloud_to_laserscan"
pkill -9 -f "ground_segmentation"
pkill -9 -f "imu_complementary"
pkill -9 -f "complementary_filter"
pkill -9 -f "fake_vel_transform"
pkill -9 -f "hik_camera"
pkill -9 -f "decision_node"
pkill -9 -f "pose_initializer"

# Kill any remaining processes with NJU-algorithm path
pkill -9 -f "/home/nvidia/NJU-algorithm"

echo "Done!"
