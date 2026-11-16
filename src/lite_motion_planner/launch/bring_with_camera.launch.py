"""Standalone arm test: start the camera, then the existing arm bringup."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    perception = get_package_share_directory('pose_estimator')
    planner = get_package_share_directory('lite_motion_planner')
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'camera_switch.launch.py')),
        launch_arguments={
            'camera_config_file': LaunchConfiguration('camera_config_file'),
        }.items(),
    )
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(planner, 'launch', 'bring_launch.launch.py')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('camera_config_file', default_value=os.path.join(
            perception, 'config', 'camera_switch.yaml')),
        DeclareLaunchArgument('arm_start_delay', default_value='3.0',
                              description='Seconds before starting the arm bringup'),
        GroupAction(actions=[camera]),
        TimerAction(period=LaunchConfiguration('arm_start_delay'),
                    actions=[arm]),
    ])
