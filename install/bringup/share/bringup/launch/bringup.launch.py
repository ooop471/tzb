"""Complete automatic mission algorithms; sensor and serial drivers run separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    state = get_package_share_directory('rover_state')
    return LaunchDescription([
        DeclareLaunchArgument('navigation_only', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mission_params', default_value=os.path.join(state, 'config', 'mission.yaml')),
        DeclareLaunchArgument('target_class', default_value='blue_block'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(state, 'launch', 'state.launch')),
            launch_arguments={
                'start_sensors': 'false',
                'navigation_only': LaunchConfiguration('navigation_only'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mission_params': LaunchConfiguration('mission_params'),
                'target_class': LaunchConfiguration('target_class'),
            }.items()),
    ])
