from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    nav_pkg_share = '/home/nvidia/NJU-algorithm/navigation_task/install/rm_nav_bringup/share/rm_nav_bringup'
    camera_pkg_share = '/home/nvidia/NJU-algorithm/vision_task/hiki_ros2/install/hik_camera/share/hik_camera'
    decision_pkg_share = '/home/nvidia/NJU-algorithm/decision_task/install/decision/share/decision'

    nav_launch_path = os.path.join(nav_pkg_share, 'launch', 'bringup_real.launch.py')
    camera_launch_path = os.path.join(camera_pkg_share, 'launch', 'hik_camera.launch.py')
    decision_launch_path = os.path.join(decision_pkg_share, 'launch', 'pose_init_and_decision.launch.py')

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        launch_arguments={
            'world': 'new_map',
            'mode': 'nav',
            'lio': 'fastlio',
            'localization': 'amcl',
            'lio_rviz': 'False',
            'nav_rviz': 'False'
        }.items()
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'publish_camera_info': 'false',
            'enable_timing_log': 'true'
        }.items()
    )

    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(decision_launch_path)
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='DECISION_PKG_DIR',
            value='/home/nvidia/NJU-algorithm/decision_task/install/decision'
        ),
        SetEnvironmentVariable(
            name='AMENT_PREFIX_PATH',
            value='/home/nvidia/NJU-algorithm/decision_task/install/decision:/home/nvidia/NJU-algorithm/decision_task/install:' + os.environ.get('AMENT_PREFIX_PATH', '')
        ),
        nav_launch,
        camera_launch,
        TimerAction(
            actions=[decision_launch],
            period=3.0
        )
    ])
