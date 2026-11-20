"""Internal navigation assembly shared by exploration and mission entry points."""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def align_exploration_velocity_limits(params):
    """Make MPPI predict the linear speed actually allowed by the smoother."""
    controller = params['controller_server']['ros__parameters']
    smoother = params['velocity_smoother']['ros__parameters']
    limits = smoother['max_velocity']
    if len(limits) != 3 or not all(math.isfinite(float(v)) for v in limits):
        raise ValueError('velocity_smoother.max_velocity needs three finite values')
    if float(limits[0]) <= 0 or float(limits[2]) <= 0:
        raise ValueError('Exploration smoother linear/angular limits must be positive')
    for name in controller['controller_plugins']:
        plugin = controller[name]
        if plugin.get('plugin') != 'nav2_mppi_controller::MPPIController':
            continue
        plugin['vx_max'] = min(float(plugin['vx_max']), float(limits[0]))
        plugin['wz_max'] = min(float(plugin['wz_max']), float(limits[2]))


def prepare_navigation(context):
    """Prepare an installed BT configuration and a map without a known obstacle prior."""
    share = get_package_share_directory('bringup')
    directory = tempfile.mkdtemp(prefix='standalone_exploration_')
    with open(LaunchConfiguration('nav_params_file').perform(context)) as stream:
        params = yaml.safe_load(stream)
    align_exploration_velocity_limits(params)
    cave = LaunchConfiguration('prepared_cave_mode').perform(context) == 'true'
    params['controller_server']['ros__parameters']['FollowPath']['CorridorCritic']['enabled'] = cave
    bt = params['bt_navigator']['ros__parameters']
    for key, filename in (
        ('default_nav_to_pose_bt_xml', 'navigate_to_point_w_recovery.xml'),
        ('default_nav_through_poses_bt_xml', 'navigate_through_pose_w_recovery.xml'),
    ):
        bt[key] = os.path.join(share, 'behavior_tree', filename)
    params_path = os.path.join(directory, 'nav2.yaml')
    with open(params_path, 'w') as stream:
        yaml.safe_dump(params, stream)
    local_map = params['local_costmap']['local_costmap']['ros__parameters']
    footprint = yaml.safe_load(local_map['footprint'])
    padding = float(local_map.get('footprint_padding', 0.01))
    xs, ys = [float(p[0]) for p in footprint], [float(p[1]) for p in footprint]
    if len(footprint) != 4 or not all(math.isfinite(v) for v in xs + ys + [padding]) or padding < 0:
        raise ValueError('Direct center path requires the configured rectangular robot footprint')
    if set(zip(xs, ys)) != {(x,y) for x in (min(xs), max(xs)) for y in (min(ys), max(ys))} or abs(min(ys)+max(ys)) > 1e-6:
        raise ValueError('Direct center path requires an axis-aligned footprint symmetric about body Y')
    managed = LaunchConfiguration('managed_mode').perform(context) == 'true'
    actions = [SetLaunchConfiguration('prepared_nav_params', params_path),
               SetLaunchConfiguration('prepared_recovery_output',
                   '/mission/recovery_cmd_vel' if managed else 'cmd_vel_nav')]
    for name, value in {'front':max(xs)+padding, 'rear':-min(xs)+padding,
                        'half_width':max(abs(y) for y in ys)+padding,
                        'side_margin':params['controller_server']['ros__parameters']['FollowPath']['CorridorCritic']['side_margin']}.items():
        actions.append(SetLaunchConfiguration('prepared_body_' + name, str(value)))
    supplied_map = LaunchConfiguration('map').perform(context)
    if supplied_map:
        return actions + [SetLaunchConfiguration('prepared_map', supplied_map)]
    width = float(LaunchConfiguration('arena_width').perform(context))
    height = float(LaunchConfiguration('arena_height').perform(context))
    if not all(math.isfinite(v) and 0 < v <= 500 for v in (width, height)):
        raise ValueError('Arena width and height must be finite, positive and <= 500 m')
    resolution = 0.1
    image = os.path.join(directory, 'unknown.pgm')
    cols, rows = math.ceil(width / resolution), math.ceil(height / resolution)
    with open(image, 'wb') as stream:
        stream.write(f'P5\n{cols} {rows}\n255\n'.encode())
        stream.write(bytes([205]) * (cols * rows))
    map_path = os.path.join(directory, 'unknown.yaml')
    with open(map_path, 'w') as stream:
        yaml.safe_dump(dict(image=image, mode='trinary', resolution=resolution,
                            origin=[-width / 2, -height / 2, 0.0], negate=0,
                            occupied_thresh=0.65, free_thresh=0.25), stream)
    return actions + [SetLaunchConfiguration('prepared_map', map_path)]


def select_path_source(context):
    use_nav2 = LaunchConfiguration('use_nav2_planner').perform(context) == 'true'
    nav_grid = LaunchConfiguration('use_nav2_collision_grid').perform(context) == 'true'
    managed = LaunchConfiguration('managed_mode').perform(context) == 'true'
    cave = LaunchConfiguration('cave_mode').perform(context)
    enabled = cave == 'true' or (cave == 'auto' and not managed and use_nav2 and nav_grid)
    if enabled and not (use_nav2 and nav_grid):
        raise ValueError('Cave mode requires the Nav2 planner and collision grid')
    return [SetLaunchConfiguration('tare_path_topic',
            '/explore/tare_control_path' if use_nav2 else '/explore/control_path'),
            SetLaunchConfiguration('prepared_cave_mode', 'true' if enabled else 'false')]


def stop_if_planner_exits(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason='Exploration Nav2 planner exited'))]


def stop_if_follower_exits(event, context):
    # During Ctrl+C the follower also exits; do not enqueue a second shutdown then.
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason='Exploration path follower exited'))]


def navigation_actions():
    share = get_package_share_directory('bringup')
    nav_share = get_package_share_directory('rm_navigation')
    explore_share = get_package_share_directory('rover_explore')
    clock = LaunchConfiguration('use_sim_time')
    clock_params = {'use_sim_time': ParameterValue(clock, value_type=bool)}

    follower = Node(
        package='rover_explore', executable='exploration_path_follower',
        name='exploration_path_follower', output='screen',
        parameters=[LaunchConfiguration('follower_params'), clock_params,
                    {'managed_mode': ParameterValue(LaunchConfiguration('managed_mode'), value_type=bool)}])
    planner = Node(
        package='rover_explore', executable='exploration_nav2_planner',
        name='exploration_nav2_planner', output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav2_planner')),
        parameters=[LaunchConfiguration('planner_params'), clock_params,
                    {'corridor_enabled': ParameterValue(LaunchConfiguration('prepared_cave_mode'), value_type=bool)}])
    nav_args = {
        'use_sim_time': clock,
        'autostart': 'true',
        'map': LaunchConfiguration('prepared_map'),
        'params_file': LaunchConfiguration('prepared_nav_params'),
        'use_composition': 'True',
        'use_respawn': 'False',
        'nav_rviz': 'false',
        'container_name': 'nav2_container',
        'cmd_vel_output': LaunchConfiguration('cmd_vel_output'),
        'recovery_cmd_vel_output': LaunchConfiguration('prepared_recovery_output'),
    }
    return [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_localization', default_value='true',
                              description='Start IMU preprocessing and Fast-LIO in this launch'),
        DeclareLaunchArgument('managed_mode', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('cave_mode', default_value='auto', choices=['auto', 'true', 'false'],
                              description='auto enables centered cave goals for standalone exploration'),
        DeclareLaunchArgument('cmd_vel_output', default_value='cmd_vel'),
        DeclareLaunchArgument('nav_params_file', default_value=os.path.join(
            share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('explore_params', default_value=os.path.join(
            explore_share, 'config', 'rover.yaml')),
        DeclareLaunchArgument('fast_lio_config_file', default_value='mid360.yaml',
                              description='Fast-LIO config file name under fast_lio/config/'),
        DeclareLaunchArgument('follower_params', default_value=os.path.join(
            explore_share, 'config', 'exploration_follower.yaml')),
        DeclareLaunchArgument('use_nav2_planner', default_value='true',
                              choices=['true', 'false'],
                              description='Nav2 plans to the TARE target; false restores TARE path transport'),
        DeclareLaunchArgument('use_nav2_collision_grid', default_value='true',
                              choices=['true', 'false'],
                              description='Use Nav2 collision maps for exploration viewpoints'),
        DeclareLaunchArgument('planner_params', default_value=os.path.join(
            explore_share, 'config', 'exploration_planner.yaml')),
        DeclareLaunchArgument('map', default_value='',
                              description='Empty uses an unknown map; otherwise use this map YAML'),
        DeclareLaunchArgument('arena_width', default_value='100.0'),
        DeclareLaunchArgument('arena_height', default_value='100.0'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        OpaqueFunction(function=select_path_source),
        OpaqueFunction(function=prepare_navigation),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='exploration_map_to_odom', output='log',
             arguments=['--x', LaunchConfiguration('start_x'),
                        '--y', LaunchConfiguration('start_y'),
                        '--yaw', LaunchConfiguration('start_yaw'),
                        '--frame-id', 'map', '--child-frame-id', 'odom_init'],
             parameters=[clock_params]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='exploration_base_to_livox', output='log',
             arguments=['--frame-id', 'base_frame', '--child-frame-id', 'livox_frame'],
             parameters=[clock_params]),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('start_localization')),
            actions=[
            Node(package='imu_tools', executable='imu_transform_node',
                 name='imu_transform', output='screen',
                 parameters=[os.path.join(get_package_share_directory('imu_tools'),
                                          'config', 'imu_transform.yaml'), clock_params]),
            Node(package='imu_tools', executable='imu_complementary_filter_node',
                 name='imu_complementary_filter', output='screen',
                 parameters=[clock_params, {'do_bias_estimation': True, 'do_adaptive_gain': True,
                                           'use_mag': False, 'gain_acc': 0.01, 'gain_mag': 0.01}],
                 remappings=[('/imu/data_raw', '/livox/imu_aft_tf')]),
            Node(package='fast_lio', executable='fastlio_mapping', output='screen',
                 parameters=[
                     os.path.join(get_package_share_directory('fast_lio'), 'config', 'mid360.yaml'),
                     {'use_sim_time': False},
                     clock_params]),
            ],
        ),
        Node(package='terrain_analysis', executable='terrainAnalysis',
             name='terrain_analysis', output='screen',
             parameters=[os.path.join(get_package_share_directory('terrain_analysis'),
                                      'config', 'terrain_analysis_plus.yaml'), clock_params]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            nav_share, 'launch', 'bringup_rm_navigation.py')), launch_arguments=nav_args.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            nav_share, 'launch', 'map_server_launch.py')), launch_arguments=nav_args.items()),
        Node(package='rover_explore', executable='rover_explore_node',
             name='rover_explore', output='screen',
             remappings=[('/explore/control_path', LaunchConfiguration('tare_path_topic'))],
             parameters=[LaunchConfiguration('explore_params'), clock_params, {
                 'kAutoStart': False, 'kRushHome': False,
                 'navigation_grid.enabled': ParameterValue(
                     LaunchConfiguration('use_nav2_collision_grid'), value_type=bool),
                 'corridor.body_front': ParameterValue(LaunchConfiguration('prepared_body_front'), value_type=float),
                 'corridor.body_rear': ParameterValue(LaunchConfiguration('prepared_body_rear'), value_type=float),
                 'corridor.body_half_width': ParameterValue(LaunchConfiguration('prepared_body_half_width'), value_type=float),
                 'corridor.side_margin': ParameterValue(LaunchConfiguration('prepared_body_side_margin'), value_type=float),
                 'corridor.enabled': ParameterValue(
                     LaunchConfiguration('prepared_cave_mode'), value_type=bool),
                 'kNoExplorationReturnHome': True,
             }]),
        RegisterEventHandler(OnProcessExit(
            target_action=follower,
            on_exit=stop_if_follower_exits)),
        RegisterEventHandler(OnProcessExit(
            target_action=planner, on_exit=stop_if_planner_exits)),
        planner,
        follower,
    ]
