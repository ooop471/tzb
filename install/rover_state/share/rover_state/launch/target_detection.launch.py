"""Exploration detection only: no registration, planner or arm command process."""
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def locator(context):
        # Resolve before wrapping: a flat list of substitutions is concatenated
        # into a scalar by launch_ros, rather than kept as a string array.
        target_class = LaunchConfiguration('target_class').perform(context)
        return [Node(package='pose_estimator', executable='far_target_locator', name='far_target_locator',
                     parameters=[{'target_classes': [target_class],
                                  'max_target_range': ParameterValue(LaunchConfiguration('target_max_range'), value_type=float)}],
                     output='screen')]

    def exited(event, context):
        return [] if context.is_shutdown else [EmitEvent(event=Shutdown(reason="Exploration detection process exited; supervisor will restart it"))]
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=exited)),
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('target_class', default_value='blue_block'),
        DeclareLaunchArgument('target_max_range', default_value='1.5'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('yolo_node'), 'launch', 'yolo_node.launch.py')),
            launch_arguments={'model_path': LaunchConfiguration('model_path'),
                              'target_class': LaunchConfiguration('target_class')}.items()),
        OpaqueFunction(function=locator)
    ])
