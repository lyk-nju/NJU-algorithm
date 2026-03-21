from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = '/home/nvidia/NJU-algorithm/decision_task/config/decision.yaml'
    
    return LaunchDescription([
        Node(
            package='decision',
            executable='decision_node',
            name='decision_node',
            parameters=[config_file],
            output='screen'
        )
    ])
