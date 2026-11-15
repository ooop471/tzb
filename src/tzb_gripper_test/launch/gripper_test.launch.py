from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'command', default_value='-1',
            description='-1 open, 1/2/3 closed mode 1/2/3'),
        DeclareLaunchArgument(
            'topic', default_value='/tzb_catch/gripper_command'),
        DeclareLaunchArgument(
            'publish_hz', default_value='10.0'),
        Node(
            package='tzb_gripper_test',
            executable='gripper_test',
            name='gripper_test',
            output='screen',
            parameters=[{
                'command': ParameterValue(LaunchConfiguration('command'), value_type=int),
                'topic': LaunchConfiguration('topic'),
                'publish_hz': ParameterValue(LaunchConfiguration('publish_hz'), value_type=float),
            }],
        ),
    ])
