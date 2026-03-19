from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_dir = os.getenv('DECISION_PKG_DIR', '/home/nvidia/NJU-algorithm/decision_task/install/decision')
    pose_init_config = os.path.join(pkg_dir, 'share', 'decision', 'config', 'pose_init.yaml')
    patrol_sim_config = os.path.join(pkg_dir, 'share', 'decision', 'config', 'patrol_sim.yaml')

    if not os.path.exists(pose_init_config):
        pose_init_config = '/home/nvidia/NJU-algorithm/decision_task/config/pose_init.yaml'
    if not os.path.exists(patrol_sim_config):
        patrol_sim_config = '/home/nvidia/NJU-algorithm/decision_task/config/patrol_sim.yaml'

    return LaunchDescription([
        Node(
            package='decision',
            executable='pose_initializer_node',
            name='pose_initializer_node',
            parameters=[pose_init_config],
            output='screen'
        ),
        Node(
            package='decision',
            executable='patrol_sim_node',
            name='patrol_sim_node',
            parameters=[patrol_sim_config],
            output='screen'
        )
    ])
