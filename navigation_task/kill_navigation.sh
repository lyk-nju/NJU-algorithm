#!/bin/bash

set -u

patterns=(
    "navigation_task"
    "rm_nav_bringup"
    "rm_navigation"
    "nav2"
    "amcl"
    "map_server"
    "lifecycle_manager"
    "fast_lio"
    "livox_ros_driver2"
    "livox_lidar"
    "imu_complementary_filter"
    "complementary_filter"
    "linefit_ground_segmentation_ros"
    "ground_segmentation_node"
    "pointcloud_to_laserscan"
    "fake_vel_transform"
    "robot_state_publisher"
    "pose_initializer"
)

echo "Stopping navigation-related processes..."

for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
done

sleep 1

for pattern in "${patterns[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
done

echo "Done. Remaining matched processes:"
pgrep -af "navigation_task|rm_nav_bringup|rm_navigation|nav2|amcl|map_server|lifecycle_manager|fast_lio|livox_ros_driver2|livox_lidar|imu_complementary_filter|complementary_filter|linefit_ground_segmentation_ros|ground_segmentation_node|pointcloud_to_laserscan|fake_vel_transform|robot_state_publisher|pose_initializer" || true
