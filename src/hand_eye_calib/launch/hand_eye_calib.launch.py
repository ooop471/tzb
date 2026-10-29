"""Launch the interactive hand-eye capture node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('hand_eye_calib'), 'config', 'hand_eye_calib.yaml'])

    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('ee_frame', default_value='tcp_link'),
        DeclareLaunchArgument('board_type', default_value='chessboard'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_100'),
        DeclareLaunchArgument('squares_x', default_value='8'),
        DeclareLaunchArgument('squares_y', default_value='11'),
        DeclareLaunchArgument('square_length', default_value='0.03'),
        DeclareLaunchArgument('marker_length', default_value='0.0225'),
        DeclareLaunchArgument('session_root', default_value=''),
    ]

    overrides = {
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'base_frame': LaunchConfiguration('base_frame'),
        'ee_frame': LaunchConfiguration('ee_frame'),
        'board_type': LaunchConfiguration('board_type'),
        'dictionary': LaunchConfiguration('dictionary'),
        'squares_x': _typed('squares_x', int),
        'squares_y': _typed('squares_y', int),
        'square_length': _typed('square_length', float),
        'marker_length': _typed('marker_length', float),
        'session_root': LaunchConfiguration('session_root'),
    }

    node = Node(
        package='hand_eye_calib',
        executable='hand_eye_calibration',
        name='hand_eye_calib',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides],
    )

    return LaunchDescription(arguments + [node])
