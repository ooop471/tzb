#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Spawn tzb_description in Gazebo Classic.

ROS 2 replacement for the former ROS 1 ``gazebo.launch``.
Requires Gazebo Classic 11 + gazebo_ros_pkgs:

    sudo apt install ros-humble-gazebo-ros-pkgs

Usage:
    ros2 launch tzb_description gazebo.launch.py
    ros2 launch tzb_description gazebo.launch.py paused:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('tzb_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    default_urdf = os.path.join(pkg_share, 'urdf', 'tzb_description.urdf')
    default_world = os.path.join(gazebo_share, 'worlds', 'empty.world')

    declare_urdf = DeclareLaunchArgument(
        'urdf', default_value=default_urdf,
        description='Absolute path of the URDF/Xacro file to spawn.')
    declare_entity = DeclareLaunchArgument(
        'entity', default_value='tzb_description',
        description='Name of the model inside Gazebo.')
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Absolute path of the Gazebo world file.')
    declare_z = DeclareLaunchArgument(
        'z', default_value='0.05',
        description='Spawn height in metres.')
    declare_paused = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Start the Gazebo server paused.')
    declare_gazebo_gui = DeclareLaunchArgument(
        'gazebo_gui', default_value='true',
        description='Start the Gazebo client (gzclient).')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf')]),
        value_type=str)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': LaunchConfiguration('paused'),
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # The URDF has no base_footprint link, so publish it as a child frame of
    # base_link (same behaviour as the ROS 1 launch file).
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': True}],
    )

    # The URDF defines no <gazebo> plugins / transmissions yet, so Gazebo does
    # not publish /joint_states for this model. This publisher keeps the TF
    # tree complete by reporting every joint at its zero position; drop it once
    # real joint state / controller plugins are added.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', LaunchConfiguration('entity'),
            '-z', LaunchConfiguration('z'),
        ],
    )

    return LaunchDescription([
        declare_urdf,
        declare_entity,
        declare_world,
        declare_z,
        declare_paused,
        declare_gazebo_gui,
        gzserver,
        gzclient,
        robot_state_publisher,
        base_footprint_tf,
        joint_state_publisher,
        spawn_entity,
    ])
