"""Observe the complete mission without starting robot algorithms or drivers."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    with open(os.path.join(get_package_share_directory('bringup'), 'config', 'nav2_params.yaml')) as stream:
        costmap = yaml.safe_load(stream)['local_costmap']['local_costmap']['ros__parameters']
    body = {'body_frame': costmap['robot_base_frame'],
            'body_footprint': ParameterValue(costmap['footprint'], value_type=str)}
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            get_package_share_directory('bringup'), 'rviz', 'state_visual.rviz')),
        Node(package='bringup', executable='state_visual_status.py',
             name='state_visual_data', parameters=[clock, body], output='screen'),
        Node(package='rviz2', executable='rviz2', name='state_visual',
             arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[clock], output='screen'),
    ])
