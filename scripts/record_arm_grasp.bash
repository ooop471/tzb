#!/usr/bin/env bash
# Record only; never starts drivers, planners, or arm tasks.
set -eo pipefail
arm_record_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$arm_record_ws/install/setup.bash"
arm_record_root="${ARM_BAG_ROOT:-$HOME/bags}"
arm_record_output="$arm_record_root/arm_grasp_$(date +%Y%m%d_%H%M%S)"
arm_record_context="${arm_record_output}_context"
arm_record_qos="$arm_record_ws/scripts/arm_record_qos.yaml"
arm_record_topics=(
  /tf /tf_static /rosout /parameter_events
  /camera/color/image_raw /camera/color/camera_info
  /camera/depth/image_rect_raw /camera/depth/camera_info /camera/depth/color/points
  /yolo/detections /yolo/annotated_image
  /pose_estimator/object_pose /pose_estimation_node/roi_cloud /pose_estimation_node/coarse_pose
  /pose_estimation_node/model_marker /pose_estimation_node/coarse_model_marker
  /joint_states /serial/arm_joint_state /tzb_catch/arm_joint_command /tzb_catch/gripper_command
  /joint_trajectory_controller/joint_trajectory /display_planned_path
  /planning_scene /monitored_planning_scene /lite_motion_planner/collision_markers
  /tzb_catch/e3_marker /tzb_catch/reference_frames /tzb_catch/frozen_observe_camera_points
  /tzb_catch/start /tzb_catch/result /navigation_goal_status
  /arm_task/command /arm_task/state /perception/navigation_status
)
arm_record_command=(ros2 bag record --include-hidden-topics --qos-profile-overrides-path "$arm_record_qos"
  --max-bag-size 2147483648 -o "$arm_record_output" "${arm_record_topics[@]}")
if [[ "${1:-}" == --dry-run && $# -eq 1 ]]; then
  printf 'Topics: %s\n' "${#arm_record_topics[@]}"
  printf '%q ' "${arm_record_command[@]}"; printf '\n'; exit 0
fi
[[ $# -eq 0 ]] || { printf 'Usage: %s [--dry-run]\n' "$0" >&2; exit 2; }
mkdir -p "$arm_record_context"
printf '%s\n' "${arm_record_topics[@]}" > "$arm_record_context/topics.txt"
printf 'ROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nRMW_IMPLEMENTATION=%s\n' \
  "${ROS_DOMAIN_ID:-0}" "${ROS_LOCALHOST_ONLY:-0}" "${RMW_IMPLEMENTATION:-default}" > "$arm_record_context/environment.txt"
cp -- "$arm_record_qos" "$arm_record_context/arm_record_qos.yaml"
python3 "$arm_record_ws/scripts/collect_arm_record_context.py" "$arm_record_ws" "$arm_record_context" --watch &
arm_record_collector=$!
"${arm_record_command[@]}" &
arm_record_pid=$!
arm_record_cleanup() {
  trap - EXIT INT TERM
  kill -INT "$arm_record_pid" 2>/dev/null || true
  wait "$arm_record_pid" 2>/dev/null || true
  kill -TERM "$arm_record_collector" 2>/dev/null || true
  wait "$arm_record_collector" 2>/dev/null || true
  python3 "$arm_record_ws/scripts/collect_arm_record_context.py" "$arm_record_ws" "$arm_record_context"
  printf '\nBag: %s\nParameters and logs: %s\n' "$arm_record_output" "$arm_record_context"
}
trap arm_record_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'Bag: %s\nContext: %s\nStart the arm launch in another terminal. Ctrl+C finishes recording.\n' \
  "$arm_record_output" "$arm_record_context"
wait "$arm_record_pid"
