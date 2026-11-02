from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_task_bridge',
            executable='arm_task_bridge',
            name='arm_task_bridge',
            output='screen',
        ),
    ])
