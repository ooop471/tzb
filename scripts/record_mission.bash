#!/usr/bin/env bash
set -eo pipefail
export NAV_MISSION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash -c '#!/usr/bin/env bash
# Keep the two different rm_serial_driver packages in separate environments.
set -eo pipefail
nav_root="$1"; shift
arm_root="${ARM_WS:-$(dirname "$nav_root")/tzb_ws}"
chassis_root="${CHASSIS_WS:-$(dirname "$nav_root")/serial_t}"
scope="${1:?environment scope required}"; shift
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
case "$scope" in
  arm|camera)
    cd "$arm_root"
    if [[ -r scripts/local_deps.bash ]]; then source scripts/local_deps.bash; fi
    source install/setup.bash ;;
  nav)
    if [[ -r "$arm_root/scripts/local_deps.bash" ]]; then source "$arm_root/scripts/local_deps.bash"; fi
    source "$arm_root/install/setup.bash"
    source "$nav_root/.deps/livox-sdk2/setup.bash"
    source "$nav_root/install/setup.bash" ;;
  chassis) source "$chassis_root/install/setup.bash" ;;
  *) printf '"'"'Unknown environment: %s\n'"'"' "$scope" >&2; exit 2 ;;
esac
# Use ROS'"'"'s default domain; never assign a custom domain ID.
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
cd "$nav_root"
exec "$@"
' mission-env "$NAV_MISSION_ROOT" nav python3 - "$@" <<'RECORD_PYTHON'
#!/usr/bin/env python3
"""Record the full mission interfaces and optionally camera data."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import shlex
import shutil
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision', action='store_true', help='Also record RGB/depth clouds and annotated images (large)')
    parser.add_argument('--output', type=Path, help='New bag directory; must not already exist')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    root = Path(os.environ['NAV_MISSION_ROOT'])
    output = (args.output or Path.home() / 'bags' / datetime.now().strftime('mission_%Y%m%d_%H%M%S')).resolve()
    if output.exists():
        parser.error(f'Output already exists: {output}')
    topics = '''/tf /tf_static /odometry /path /cloud_registered /terrain_map /map
/global_costmap/costmap /local_costmap/costmap /local_costmap/costmap_raw
/local_costmap/published_footprint /plan /cmd_vel /cmd_vel_nav /mission/nav_cmd_vel /mission/recovery_cmd_vel /mission/alignment_cmd_vel
/explore/exit_detection /explore/enabled /explore/status /explore/coverage_complete /explore/waypoint
/explore/control_path /explore/tare_control_path /explore/corridor_centerline /explore/corridor_selection /explore/corridor_boundary /explore/corridor_phase /explore/reference_path
/local_path /global_path /frontier_cloud /filtered_frontier_cloud /uncovered_frontier_cloud
/viewpoint_vis_cloud /selected_viewpoint_vis_cloud /exploration/state /exploration/command
/exploration/executed_path /mission/state /mission/gate_state /mission/motion_permission
/mission/target_hint /mission/target_center /mission/approach_goal /mission/entry_goal
/mission/exit_position /mission/destination /arm_task/state /serial/arm_joint_state
/joint_states /chassis/unload_command /chassis/unload_state /perception/object_center
/perception/target_hint /perception/far_target /perception/far_target/status /perception/navigation_status
/mission/target_class /mission/locked_target_class /pose_estimator/object_pose /tzb_catch/arm_joint_command /tzb_catch/gripper_command
/joint_trajectory_controller/joint_trajectory /tzb_catch/result
/yolo/detections /livox/lidar /livox/imu /imu/data /rosout /parameter_events'''.split()
    for action in ('compute_path_to_pose', 'compute_path_through_poses', 'follow_path', 'spin', 'backup', 'arm_task/execute'):
        topics += [f'/{action}/_action/status', f'/{action}/_action/feedback']
    if args.vision:
        topics += ['/camera/color/image_raw', '/camera/color/camera_info', '/camera/depth/image_rect_raw',
                   '/camera/depth/color/points', '/yolo/annotated_image']
    context = output.with_name(output.name + '_context')
    qos_path = context / 'record_qos.yaml'
    cmd = ['ros2', 'bag', 'record', '--include-hidden-topics', '--qos-profile-overrides-path', str(qos_path),
           '--max-bag-size', '2147483648', '-o', str(output), *topics]
    if args.dry_run:
        print(shlex.join(cmd))
        return
    context.mkdir(parents=True, exist_ok=False)
    qos = {}
    latched = ['/tf_static', '/map', '/global_costmap/costmap', '/local_costmap/costmap',
               '/explore/status', '/explore/enabled', '/explore/corridor_centerline', '/explore/corridor_boundary', '/explore/corridor_phase', '/explore/reference_path', '/mission/state',
               '/tzb_catch/gripper_command', '/mission/target_class', '/mission/locked_target_class', '/mission/entry_goal', '/mission/exit_position', '/mission/destination', '/mission/approach_goal']
    for topic in latched:
        qos[topic] = dict(history='keep_last', depth=10, reliability='reliable', durability='transient_local')
    for topic in ['/cmd_vel', '/cmd_vel_nav', '/mission/nav_cmd_vel', '/mission/recovery_cmd_vel', '/odometry', '/cloud_registered', '/terrain_map',
                  '/livox/lidar', '/livox/imu', '/imu/data', '/tzb_catch/arm_joint_command', '/serial/arm_joint_state'] + [t for t in topics if t.startswith('/camera/')]:
        qos[topic] = dict(history='keep_last', depth=20, reliability='best_effort', durability='volatile')
    qos_path.write_text(yaml.safe_dump(qos))
    for relative in ('src/bringup/config/nav2_params.yaml', 'src/state/config/mission.yaml',
                     'src/explore/config/rover.yaml', 'src/explore/config/exploration_planner.yaml',
                     'src/explore/config/exploration_follower.yaml'):
        shutil.copy2(root / relative, context / Path(relative).name)
    arm_root = Path(os.environ.get('ARM_WS', root.parent / 'tzb_ws'))
    for relative, name in [('src/lite_motion_planner/config/tzb_planner.yaml', 'arm_planner.yaml'),
                           ('src/pose_estimator/config/params.yaml', 'perception.yaml'),
                           ('src/pose_estimator/config/camera_switch.yaml', 'camera.yaml'),
                           ('src/lite_motion_planner/launch/mission_arm.launch.py', 'mission_arm.launch.py')]:
        source = arm_root / relative
        if source.is_file():
            shutil.copy2(source, context / name)
    (context / 'command.txt').write_text(shlex.join(cmd) + '\n')
    (context / 'environment.txt').write_text('\n'.join(f'{k}={os.environ.get(k,"unset")}' for k in
                                                     ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION')) + '\n')
    print(f'Recording: {output}\nContext: {context}\nCtrl+C to finalize bag.', flush=True)
    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    main()

RECORD_PYTHON
