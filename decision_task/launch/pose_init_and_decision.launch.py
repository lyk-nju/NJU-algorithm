from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    pkg_dir = os.getenv('DECISION_PKG_DIR', '/home/nvidia/NJU-algorithm/decision_task/install/decision')
    
    pose_config_file = os.path.join(pkg_dir, 'share', 'decision', 'config', 'pose_init.yaml')
    if not os.path.exists(pose_config_file):
        pose_config_file = '/home/nvidia/NJU-algorithm/decision_task/config/pose_init.yaml'
    
    decision_config_file = os.path.join(pkg_dir, 'share', 'decision', 'config', 'decision.yaml')
    if not os.path.exists(decision_config_file):
        decision_config_file = '/home/nvidia/NJU-algorithm/decision_task/config/decision.yaml'
    
    pose_initializer_node = Node(
        package='decision',
        executable='pose_initializer_node',
        name='pose_initializer_node',
        parameters=[pose_config_file],
        output='screen'
    )
    
    decision_node = Node(
        package='decision',
        executable='decision_node',
        name='decision_node',
        parameters=[decision_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        pose_initializer_node,
        TimerAction(
            actions=[decision_node],
            period=2.0
        )
    ])
