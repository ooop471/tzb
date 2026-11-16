import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _pkg_launch(package_name, launch_name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', launch_name)
    )


def _defer_arm_include(context, period, action):
    """Resolve arguments before the enclosing mission launch scope is restored."""
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    if action.condition is not None and not action.condition.evaluate(context):
        return []
    arguments = [(name, perform_substitutions(context, normalize_to_list_of_substitutions(value)))
                 for name, value in action.launch_arguments]
    return [TimerAction(period=period, actions=[IncludeLaunchDescription(
        action.launch_description_source, launch_arguments=arguments)])]


def generate_launch_description():
    pose_estimator_share = get_package_share_directory('pose_estimator')
    lite_motion_share = get_package_share_directory('lite_motion_planner')
    serial_share = get_package_share_directory('rm_serial_driver')
    nav2arm_share = get_package_share_directory('nav2arm')

    pose_params_file = os.path.join(pose_estimator_share, 'config', 'params.yaml')
    camera_config_file = os.path.join(pose_estimator_share, 'config', 'camera_switch.yaml')
    rviz_config_file = os.path.join(lite_motion_share, 'config', 'lite_motion_debug_safe.rviz')
    serial_params_file = os.path.join(serial_share, 'config', 'serial_params.yaml')

    nuc_arg = DeclareLaunchArgument(
        'NUC',
        default_value='false',
        description='false=Jiaolong laptop profile, true=NUC profile',
    )
    pose_params_file_arg = DeclareLaunchArgument(
        'pose_params_file',
        default_value=pose_params_file,
        description='Parameters file for pose_estimator',
    )
    camera_config_file_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=camera_config_file,
        description='YAML selecting Orbbec Gemini 336L by default (D435i also supported)',
    )
    planner_params_file_arg = DeclareLaunchArgument(
        'planner_params_file',
        default_value='',
        description='Optional planner YAML override; empty selects it from robot_model',
    )
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='tzb_description',
        description='Robot model: tzb_description (default) or engineer_0520_description',
    )
    wait_for_nav_go_finish_arg = DeclareLaunchArgument(
        'wait_for_nav_go_finish',
        default_value='true',
        description='true waits for nav_go_finish; false starts the arm task immediately',
    )
    use_nav2arm_arg = DeclareLaunchArgument(
        'use_nav2arm',
        default_value='false',
        description='Start the legacy nav2arm state bridge',
    )
    use_arm2nav_arg = DeclareLaunchArgument(
        'use_arm2nav',
        default_value='false',
        description='Optional sentry-mode arm2nav bridge; disabled for the legacy navigation-status flow',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Start arm task immediately; false waits for /navigation_goal_status=1',
    )
    listen_navigation_goal_status_arg = DeclareLaunchArgument(
        'listen_navigation_goal_status',
        default_value='',
        description='Optional override: listen for /navigation_goal_status before starting the arm task',
    )
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_robot_state_publisher_arg = DeclareLaunchArgument(
        'use_robot_state_publisher',
        default_value='true',
    )
    use_serial_driver_arg = DeclareLaunchArgument('use_serial_driver', default_value='true')
    serial_params_file_arg = DeclareLaunchArgument(
        'serial_params_file', default_value=serial_params_file)
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=rviz_config_file)


    yolo_launch = IncludeLaunchDescription(
        _pkg_launch('yolo_node', 'yolo_node.launch.py'),
        launch_arguments={
            'NUC': LaunchConfiguration('NUC'),
            'model_path': LaunchConfiguration('model_path'),
            'target_class': LaunchConfiguration('target_class'),
            'target_classes': LaunchConfiguration('target_classes'),
            'ignored_classes': LaunchConfiguration('ignored_classes'),
        }.items(),
    )

    pose_estimator_launch = IncludeLaunchDescription(
        _pkg_launch('pose_estimator', 'pose_estimator.launch.py'),
        launch_arguments={
            'params_file': LaunchConfiguration('pose_params_file'),
            'launch_camera': 'false',
            'NUC': LaunchConfiguration('NUC'),
        }.items(),
    )

    tzb_catch_launch = IncludeLaunchDescription(
        _pkg_launch('lite_motion_planner', 'tzb_catch.launch.py'),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'autostart': LaunchConfiguration('autostart'),
            'listen_navigation_goal_status': LaunchConfiguration('listen_navigation_goal_status'),
            'params_file': LaunchConfiguration('planner_params_file'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_robot_state_publisher': LaunchConfiguration('use_robot_state_publisher'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_external_serial': LaunchConfiguration('use_external_serial'),
            'use_serial_driver': LaunchConfiguration('use_serial_driver'),
            'serial_params_file': LaunchConfiguration('serial_params_file'),
        }.items(),
    )

    nav2arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2arm_share, 'launch', 'nav2arm.launch.py')
        ),
        launch_arguments={
            'wait_for_nav_go_finish': LaunchConfiguration('wait_for_nav_go_finish'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2arm')),
    )

    arm2nav_launch = IncludeLaunchDescription(
        _pkg_launch('arm2nav', 'arm2nav.launch.py'),
        condition=IfCondition(LaunchConfiguration('use_arm2nav')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_external_serial', default_value='false'),
        DeclareLaunchArgument('model_path', default_value='/home/tzb/tzb_ws/src/best_openvino_model/blue_block_v1_640_openvino_model'),
        DeclareLaunchArgument('target_class', default_value='blue_block'),
        DeclareLaunchArgument('target_classes', default_value='blue_block'),
        DeclareLaunchArgument('ignored_classes', default_value='d'),
        nuc_arg,
        robot_model_arg,
        wait_for_nav_go_finish_arg,
        use_nav2arm_arg,
        use_arm2nav_arg,
        autostart_arg,
        listen_navigation_goal_status_arg,
        pose_params_file_arg,
        camera_config_file_arg,
        planner_params_file_arg,
        use_rviz_arg,
        use_robot_state_publisher_arg,
        use_serial_driver_arg,
        serial_params_file_arg,
        rviz_config_arg,
        OpaqueFunction(function=lambda context: _defer_arm_include(context, 1.0, yolo_launch)),
        OpaqueFunction(function=lambda context: _defer_arm_include(context, 2.0, pose_estimator_launch)),
        OpaqueFunction(function=lambda context: _defer_arm_include(context, 3.0, tzb_catch_launch)),
        OpaqueFunction(function=lambda context: _defer_arm_include(context, 3.5, nav2arm_launch)),
        OpaqueFunction(function=lambda context: _defer_arm_include(context, 3.5, arm2nav_launch)),
    ])
