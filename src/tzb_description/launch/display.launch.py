#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Display the tzb_description robot model in RViz2.

ROS 2 replacement for the former ROS 1 ``display.launch``.

Usage:
    ros2 launch tzb_description display.launch.py
    ros2 launch tzb_description display.launch.py gui:=false rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('tzb_description')
    default_urdf = os.path.join(pkg_share, 'urdf', 'tzb_description.urdf')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'tzb_description.rviz')

    declare_urdf = DeclareLaunchArgument(
        'urdf', default_value=default_urdf,
        description='Absolute path of the URDF/Xacro file to load.')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start joint_state_publisher_gui (true) or joint_state_publisher (false).')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz2 together with robot_state_publisher.')
    declare_rviz_config = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_config,
        description='Absolute path of the RViz2 configuration file.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the simulation clock (/clock) if true.')

    # xacro also accepts plain .urdf files, so both work here.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_urdf,
        declare_gui,
        declare_rviz,
        declare_rviz_config,
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz2,
    ])
