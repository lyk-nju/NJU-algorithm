from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = '/home/nvidia/NJU-algorithm/decision_task/config/patrol_sim.yaml'
    
    return LaunchDescription([
        Node(
            package='decision',
            executable='patrol_sim_node',
            name='patrol_sim_node',
            parameters=[config_file],
            output='screen'
        )
    ])
