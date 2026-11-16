"""Camera, perception and externally controlled grasp. No serial driver is launched."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

# Seconds launch waits after SIGINT before SIGTERM, and after SIGTERM before
# SIGKILL. Must exceed tzb.shutdown_home_timeout_sec + 5 s watchdog (tzb_catch)
# and tzb.shutdown_hold_timeout_sec (planning_node, serial driver).
SHUTDOWN_SIGTERM_TIMEOUT = '55'
SHUTDOWN_SIGKILL_TIMEOUT = '10'


def prepare(context):
    planner = get_package_share_directory('lite_motion_planner')
    description = get_package_share_directory('tzb_description')
    perception = get_package_share_directory('pose_estimator')
    with open(os.path.join(planner, 'config', 'tzb_planner.yaml'), encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    p = params['/**']['ros__parameters']
    p.update(urdf_path=os.path.join(description, 'urdf', 'tzb_description.urdf'),
             srdf_path=os.path.join(planner, 'config', 'tzb_description.srdf'),
             mesh_package_dirs=[description], enable_visualization=False,
             enable_joint_state_playback=False, publish_joint_states_for_visualization=False,
             launch_visualization_rviz=False, launch_robot_state_publisher=False)
    p.update({'tzb.mission_controlled': True, 'tzb.autostart': False,
              'tzb.sentry_mode': False, 'tzb.sequence_loop': False, 'tzb.replay_after_complete': False,
              'tzb.listen_navigation_goal_status': False,
              # Competition profile: one perception attempt, then recover and skip.
              'tzb.pose_estimator_wait_timeout_sec': 8.0, 'tzb.max_target_attempts': 1,
              'tzb.navigation_sensor_frame': LaunchConfiguration('navigation_sensor_frame').perform(context)})
    target_classes = [c.strip() for c in
                      LaunchConfiguration('target_classes').perform(context).split(',') if c.strip()]
    with open(os.path.join(perception, 'config', 'params.yaml'), encoding='utf-8') as stream:
        vision = yaml.safe_load(stream)
    vision['pose_estimation_node']['ros__parameters'].update(
        model_path=os.path.join(perception, 'config', 'half_block.pcd'), navigation_observations=True,
        navigation_confidence=0.1,
        # 11-07: far_target_locator is the only /mission/target_hint source (pose estimator keeps /perception/object_center)
        publish_target_hint=False, candidate_enable=False,
        target_classes=target_classes)
    directory = tempfile.mkdtemp(prefix='grasp_arm_')
    planner_path = os.path.join(directory, 'planner.yaml')
    vision_path = os.path.join(directory, 'vision.yaml')
    for path, data in [(planner_path, params), (vision_path, vision)]:
        with open(path, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream)
    camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        perception, 'launch', 'camera_switch.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={'camera_config_file': os.path.join(perception, 'config', 'camera_switch.yaml')}.items())
    detector = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('yolo_node'), 'launch', 'yolo_node.launch.py')),
        launch_arguments={'model_path': LaunchConfiguration('model_path'),
                          'target_classes': ','.join(target_classes)}.items())
    return [camera, detector,
            Node(package='pose_estimator', executable='far_target_locator',
                 name='far_target_locator', output='screen'),
            Node(package='pose_estimator', executable='pose_estimation_node',
                 name='pose_estimation_node', parameters=[vision_path], output='screen'),
            # tzb_catch homes the arm after SIGINT (bounded by
            # tzb.shutdown_home_timeout_sec) and planning_node keeps serving it
            # while the shutdown lease is fresh: launch must not escalate to
            # SIGTERM/SIGKILL before that finishes.
            Node(package='lite_motion_planner', executable='planning_node',
                 name='planning_node', parameters=[planner_path], output='screen',
                 sigterm_timeout=SHUTDOWN_SIGTERM_TIMEOUT, sigkill_timeout=SHUTDOWN_SIGKILL_TIMEOUT),
            Node(package='lite_motion_planner', executable='tzb_catch',
                 name='tzb_catch', parameters=[planner_path], output='screen',
                 sigterm_timeout=SHUTDOWN_SIGTERM_TIMEOUT, sigkill_timeout=SHUTDOWN_SIGKILL_TIMEOUT)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=os.path.join(
            get_package_share_directory('yolo_node'), 'models', 'blue_block_v1_640_openvino_model')),
        DeclareLaunchArgument('target_classes', default_value='blue_block,big_rock'),
        DeclareLaunchArgument('navigation_sensor_frame', default_value='camera_color_optical_frame'),
        OpaqueFunction(function=prepare),
    ])
