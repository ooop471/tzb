"""Start RViz with the same configuration and robot parameters as bring_launch."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SUPPORTED_ROBOT_MODELS = {
    'tzb_description': 'tzb_planner.yaml',
    'engineer_0520_description': 'engineer_0520_planner.yaml',
}


def _build_rviz(context, pkg_share):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    if robot_model not in SUPPORTED_ROBOT_MODELS:
        raise RuntimeError(f'Unsupported robot_model: {robot_model}')
    params_file = LaunchConfiguration('planner_params_file').perform(context).strip()
    if not params_file:
        params_file = os.path.join(pkg_share, 'config', SUPPORTED_ROBOT_MODELS[robot_model])
    with open(params_file, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    params = data.get('/**', {}).get('ros__parameters', {})
    descriptions = {}
    for parameter, path_key in (
        ('robot_description', 'urdf_path'),
        ('robot_description_semantic', 'srdf_path'),
    ):
        path = params.get(path_key, '')
        content = ''
        if path and os.path.exists(path):
            with open(path, encoding='utf-8') as stream:
                content = stream.read()
        descriptions[parameter] = content
    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[descriptions],
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('lite_motion_planner')
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='tzb_description',
                              description='tzb_description or engineer_0520_description'),
        DeclareLaunchArgument('planner_params_file', default_value='',
                              description='Optional planner YAML override for robot descriptions'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            pkg_share, 'config', 'lite_motion_debug_safe.rviz')),
        OpaqueFunction(function=lambda context: _build_rviz(context, pkg_share)),
    ])
