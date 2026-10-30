from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tzb_target_bridge',
            executable='target_bridge.py',
            name='tzb_target_bridge',
            output='screen',
        ),
    ])
