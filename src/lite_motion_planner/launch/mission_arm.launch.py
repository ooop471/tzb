"""Mission adapter for bring_launch; camera and serial are owned by start_mission."""
import os
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare(context):
    planner = Path(get_package_share_directory('lite_motion_planner'))
    perception = Path(get_package_share_directory('pose_estimator'))
    description = Path(get_package_share_directory('tzb_description'))
    params = yaml.safe_load((planner / 'config/tzb_planner.yaml').read_text())
    p = params['/**']['ros__parameters']
    p.update(urdf_path=str(description / 'urdf/tzb_description.urdf'),
             srdf_path=str(planner / 'config/tzb_description.srdf'),
             mesh_package_dirs=[str(description)], enable_visualization=False,
             enable_joint_state_playback=False, publish_joint_states_for_visualization=False,
             launch_visualization_rviz=False, launch_robot_state_publisher=False)
    # Grasp timing, retries, poses and calibration come from the standalone YAML.
    p.update({'tzb.mission_controlled': True, 'tzb.autostart': False,
              'tzb.sentry_mode': False, 'tzb.sequence_loop': False,
              'tzb.replay_after_complete': False, 'tzb.use_yaml_e3_target': False,
              'tzb.listen_navigation_goal_status': False,
              'tzb.arm_feedback_enabled': True, 'tzb.require_arm_feedback_for_completion': True,
              'tzb.navigation_sensor_frame': LaunchConfiguration('navigation_sensor_frame').perform(context)})
    vision = yaml.safe_load((perception / 'config/params.yaml').read_text())
    v = vision['pose_estimation_node']['ros__parameters']
    # Navigation needs observation outputs, but grasp uses the standalone cloud
    # processing and continuous registration, without a second state/time gate.
    v.update(model_path=str(perception / 'config/half_block.pcd'),
             navigation_observations=True, navigation_confidence=0.1,
             match_detection_cloud=False,
             publish_target_hint=False, candidate_enable=False,
             registration_gate_enabled=False)
    target = LaunchConfiguration('target_class').perform(context)
    if target:
        v.update(target_class=target, target_classes=[target])
    directory = Path(tempfile.mkdtemp(prefix='mission_arm_'))
    for name, data in [('planner.yaml', params), ('vision.yaml', vision)]:
        (directory / name).write_text(yaml.safe_dump(data))
    args = {'planner_params_file': str(directory / 'planner.yaml'),
            'pose_params_file': str(directory / 'vision.yaml'),
            'autostart': 'false', 'listen_navigation_goal_status': 'false',
            'use_nav2arm': 'false', 'use_arm2nav': 'false',
            'use_serial_driver': 'false', 'use_external_serial': 'true',
            'use_rviz': 'false', 'use_robot_state_publisher': 'false'}
    model = LaunchConfiguration('model_path').perform(context)
    if model:
        args['model_path'] = model
    if target:
        args.update(target_class=target, target_classes=target)
    return [GroupAction(actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(planner / 'launch/bring_launch.launch.py')), launch_arguments=args.items()),
        Node(package='pose_estimator', executable='far_target_locator',
             name='far_target_locator', output='screen',
             parameters=[{'max_target_range': float(LaunchConfiguration('target_max_range').perform(context)),
                          'target_classes': [target or 'blue_block']}]),
    ], forwarding=False)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_max_range', default_value='1.5'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('target_class', default_value='blue_block'),
        DeclareLaunchArgument('navigation_sensor_frame', default_value='camera_color_optical_frame'),
        OpaqueFunction(function=prepare),
    ])
