"""启动第二个位姿估计节点：宽口径，用于远距离发现目标。

抓取仍然由 bring_launch.launch.py 里的原节点负责（严格参数），
本 launch 只额外起一个独立节点，互不影响：

    原节点  -> /pose_estimator/object_pose        （抓取用，严格）
    本节点  -> /pose_estimator/object_pose_wide   （发现用，宽松）

用法：
    ros2 launch pose_estimator pose_estimator_wide.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pose_estimator')
    default_params = os.path.join(share, 'config', 'params_wide.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='宽口径参数文件路径',
        ),
        Node(
            package='pose_estimator',
            executable='pose_estimation_node',
            name='pose_estimation_wide',
            output='screen',
            parameters=[params_file],
        ),
        LogInfo(msg=[
            '\n============================================\n',
            '  宽口径位姿节点已启动\n',
            '  输出话题: /pose_estimator/object_pose_wide\n',
            '  参数文件: ', params_file, '\n',
            '============================================\n',
        ]),
    ])
