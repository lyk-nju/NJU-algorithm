from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 使用环境变量或默认路径
    pkg_dir = os.getenv('DECISION_PKG_DIR', '/home/nvidia/NJU-algorithm/decision_task/install/decision')
    config_file = os.path.join(pkg_dir, 'share', 'decision', 'config', 'pose_init.yaml')
    
    # 如果 install 目录不存在，使用 build 目录
    if not os.path.exists(config_file):
        config_file = '/home/nvidia/NJU-algorithm/decision_task/config/pose_init.yaml'
    
    return LaunchDescription([
        Node(
            package='decision',
            executable='pose_initializer_node',
            name='pose_initializer_node',
            parameters=[config_file],
            output='screen'
        )
    ])
