import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('mineral_nav_goal'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='mineral_nav_goal',
            executable='mineral_nav_goal',
            name='mineral_nav_goal',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
