"""Standalone exploration with MID360, Fast-LIO, terrain analysis, and Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rover_explore_launch.navigation import navigation_actions


def generate_launch_description():
    explore_share = get_package_share_directory('rover_explore')
    fast_lio_share = get_package_share_directory('fast_lio')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            explore_share, 'config', 'rover.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                fast_lio_share, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        # Enable exploration only after sensors, TF and Nav2 are ready.
        GroupAction([
            SetLaunchConfiguration('explore_params', LaunchConfiguration('params_file')),
            SetLaunchConfiguration('start_localization', 'false'),
            SetLaunchConfiguration('managed_mode', 'false'),
            SetLaunchConfiguration('use_nav2_planner', 'true'),
            SetLaunchConfiguration('use_nav2_collision_grid', 'true'),
            *navigation_actions(),
        ]),
    ])
