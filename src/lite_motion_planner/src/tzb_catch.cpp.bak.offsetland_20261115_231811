#include "lite_motion_planner/robot_model.hpp"

#include <lite_motion_msgs/srv/add_scene_object.hpp>
#include <lite_motion_msgs/srv/attach_scene_object.hpp>
#include <lite_motion_msgs/srv/plan_arm_motion.hpp>
#include <lite_motion_msgs/srv/remove_scene_object.hpp>
#include <lite_motion_msgs/srv/set_scene_collision_allowance.hpp>

#include <arm2nav_msgs/msg/arm2_nav.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_task_interfaces/action/execute_arm_task.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <limits>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <deque>
#include <optional>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace lite_motion_planner {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string normalizeAngleUnit(std::string unit) {
  std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return unit;
}

bool isDegreeUnit(const std::string& unit) {
  return unit == "deg" || unit == "degree" || unit == "degrees";
}

bool isRadianUnit(const std::string& unit) {
  return unit == "rad" || unit == "radian" || unit == "radians";
}

void degreesToRadiansInPlace(std::vector<double>& values) {
  for (double& value : values) {
    value *= kPi / 180.0;
  }
}

geometry_msgs::msg::Quaternion quaternionMsgFromRpy(double roll, double pitch, double yaw) {
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond q = rz * ry * rx;

  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

Eigen::Quaterniond quaternionEigenFromMsg(const geometry_msgs::msg::Quaternion& q_msg) {
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

geometry_msgs::msg::Pose poseMsgFromSE3(const pinocchio::SE3& tf) {
  geometry_msgs::msg::Pose out;
  out.position.x = tf.translation().x();
  out.position.y = tf.translation().y();
  out.position.z = tf.translation().z();
  const Eigen::Quaterniond q(tf.rotation());
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

pinocchio::SE3 poseMsgToSE3(const geometry_msgs::msg::Pose& pose) {
  return pinocchio::SE3(
    quaternionEigenFromMsg(pose.orientation).toRotationMatrix(),
    Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
}

double trajectoryDurationSec(const trajectory_msgs::msg::JointTrajectory& traj) {
  if (traj.points.empty()) {
    return 0.0;
  }
  const auto& last = traj.points.back().time_from_start;
  return static_cast<double>(last.sec) + static_cast<double>(last.nanosec) * 1e-9;
}

builtin_interfaces::msg::Duration durationFromSec(double seconds) {
  builtin_interfaces::msg::Duration out;
  seconds = std::max(0.0, seconds);
  out.sec = static_cast<int32_t>(std::floor(seconds));
  out.nanosec = static_cast<uint32_t>(
    std::llround((seconds - static_cast<double>(out.sec)) * 1e9));
  if (out.nanosec >= 1000000000u) {
    out.sec += 1;
    out.nanosec -= 1000000000u;
  }
  return out;
}

}  // namespace

class TzbCatchNode : public rclcpp::Node {
public:
  using ExecuteArmTask = arm_task_interfaces::action::ExecuteArmTask;
  using ExecuteArmTaskGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteArmTask>;

  TzbCatchNode()
  : Node("tzb_catch") {
    const auto urdf_path = this->declare_parameter<std::string>("urdf_path", "");
    const auto srdf_path = this->declare_parameter<std::string>("srdf_path", "");
    const auto mesh_package_dirs = this->declare_parameter<std::vector<std::string>>(
      "mesh_package_dirs", std::vector<std::string>());

    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "base_link");
    arm_group_ = this->declare_parameter<std::string>("tzb.arm_group", "tzb_group");
    tcp_frame_ = this->declare_parameter<std::string>("tzb.tcp_frame", "tcp_link");
    e3_object_id_ = this->declare_parameter<std::string>("tzb.e3_object_id", "E3");
    e3_mesh_resource_ = this->declare_parameter<std::string>(
      "tzb.e3_mesh_resource",
      "package://lite_motion_planner/config/E3.STL");
    mesh_scale_ = this->declare_parameter<std::vector<double>>(
      "tzb.mesh_scale", std::vector<double>{0.001, 0.001, 0.001});
    if (!mesh_scale_.empty() && mesh_scale_.size() != 3) {
      throw std::runtime_error("tzb.mesh_scale must contain exactly 3 values");
    }

    e3_target_frame_ = this->declare_parameter<std::string>("tzb.e3_target_frame", "base_link");
    e3_target_xyz_ = declareVec3Param("tzb.e3_target_xyz", {0.35, 0.20, 0.20});
    e3_target_rpy_ = declareVec3Param("tzb.e3_target_rpy", {0.0, 0.0, 0.0});
    use_yaml_e3_target_ = this->declare_parameter<bool>("tzb.use_yaml_e3_target", true);
    force_target_roll_ = this->declare_parameter<bool>("tzb.force_target_roll", false);
    force_target_pitch_ = this->declare_parameter<bool>("tzb.force_target_pitch", false);
    force_target_yaw_ = this->declare_parameter<bool>("tzb.force_target_yaw", false);
    adaptive_rpy_ = this->declare_parameter<bool>("tzb.adaptive_rpy", false);
    position_only_target_ = this->declare_parameter<bool>("tzb.position_only_target", false);
    const bool any_forced_target_rpy =
      force_target_roll_ || force_target_pitch_ || force_target_yaw_;
    if (any_forced_target_rpy && (adaptive_rpy_ || position_only_target_)) {
      throw std::runtime_error(
        "tzb.force_target_roll/pitch/yaw cannot be combined with tzb.adaptive_rpy or "
        "tzb.position_only_target");
    }
    if (adaptive_rpy_ && position_only_target_) {
      throw std::runtime_error(
        "tzb.adaptive_rpy and tzb.position_only_target are mutually exclusive");
    }
    adaptive_rpy_step_rad_ = this->declare_parameter<double>(
      "tzb.adaptive_rpy_step_deg", 15.0) * kPi / 180.0;
    adaptive_rpy_max_rad_ = this->declare_parameter<double>(
      "tzb.adaptive_rpy_max_deg", 45.0) * kPi / 180.0;
    adaptive_rpy_max_candidates_ = this->declare_parameter<int>(
      "tzb.adaptive_rpy_max_candidates", 64);
    if (adaptive_rpy_step_rad_ <= 0.0 || adaptive_rpy_max_rad_ < 0.0 ||
        adaptive_rpy_max_candidates_ <= 0) {
      throw std::runtime_error(
        "adaptive RPY parameters must have positive step/candidate values and nonnegative max angle");
    }
    target_rpy_unit_ = normalizeAngleUnit(
      this->declare_parameter<std::string>("tzb.target_rpy_unit", "rad"));
    if (!isRadianUnit(target_rpy_unit_) && !isDegreeUnit(target_rpy_unit_)) {
      throw std::runtime_error("tzb.target_rpy_unit must be 'rad' or 'deg'");
    }
    forced_target_roll_ = this->declare_parameter<double>("tzb.forced_target_roll", 0.0);
    forced_target_pitch_ = this->declare_parameter<double>("tzb.forced_target_pitch", 0.0);
    forced_target_yaw_ = this->declare_parameter<double>("tzb.forced_target_yaw", 0.0);
    // Cube-aware yaw (2026-11-06 real grasp): the registered model axes of a cube are arbitrary
    // (the vertical model axis may be x, y or z), so the ZYX yaw of the full detected rotation can be
    // meaningless (observed: J6 = 2.12 rad, jaws ~40 deg off the faces, grasp missed). With
    // symmetry > 0: take the model axis closest to vertical, use a horizontal model axis for the
    // face yaw, and pick the symmetric equivalent closest to the radial direction (small wrist turn).
    target_yaw_symmetry_deg_ = this->declare_parameter<double>("tzb.target_yaw_symmetry_deg", 90.0);
    // Ground-resting targets (2026-11-06 real grasp #6): the registered model-centre height of the
    // cube varied by ~5 cm between registrations of the same block on the same floor (TCP z -0.172
    // vs -0.124; the gripper closed above the block). The vehicle-frame height of a block resting on
    // the ground the vehicle stands on is constant (also on a uniform slope), so when
    // tzb.grasp_ground_z is finite the TCP target z is set to it; a visual z further than
    // tzb.grasp_ground_z_max_deviation from it is treated as an inconsistent target (rejected).
    grasp_ground_z_ = this->declare_parameter<double>("tzb.grasp_ground_z", std::numeric_limits<double>::quiet_NaN());
    grasp_ground_z_max_deviation_ = this->declare_parameter<double>("tzb.grasp_ground_z_max_deviation", 0.08);
    // 11-14 sand field: the floor height is NOT constant (block visual z -0.179..-0.204 within 30 cm on sand,
    // -0.19 on tile). Overwriting the measured z with a fixed tzb.grasp_ground_z made the gripper close up to
    // 2.5 cm below the block into the sand. grasp_use_visual_z=true keeps the measured z (the sanity gate
    // against tzb.grasp_ground_z still applies). false = old fixed-floor behaviour.
    grasp_use_visual_z_ = this->declare_parameter<bool>("tzb.grasp_use_visual_z", false);
    target_yaw_reference_offset_deg_ =
      this->declare_parameter<double>("tzb.target_yaw_reference_offset_deg", 0.0);
    if (isDegreeUnit(target_rpy_unit_)) {
      forced_target_roll_ *= kPi / 180.0;
      forced_target_pitch_ *= kPi / 180.0;
      forced_target_yaw_ *= kPi / 180.0;
    }
    pose_estimator_target_topic_ = this->declare_parameter<std::string>(
      "tzb.pose_estimator_target_topic", "/pose_estimator/object_pose");
    pose_estimator_wait_timeout_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.pose_estimator_wait_timeout_sec", 3.0));
    // 单次抓取任务里"目标获取"的最大次数；<=0 表示不限制（旧行为）。
    max_target_attempts_ = std::max(
      0,
      static_cast<int>(this->declare_parameter<int>("tzb.max_target_attempts", 5)));
    pose_estimator_continuous_required_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.pose_estimator_continuous_required_sec", 2.0));
    target_lock_max_spread_m_ = this->declare_parameter<double>(
      "tzb.target_lock_max_spread_m", 0.01);
    pose_estimator_continuous_max_gap_sec_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.pose_estimator_continuous_max_gap_sec", 0.75));
    pose_estimator_reference_frame_ = this->declare_parameter<std::string>(
      "tzb.pose_estimator_reference_frame", "");
    pose_estimator_reference_transform_ = declareMat4Param(
      "tzb.pose_estimator_reference_transform_matrix",
      { +0.90914413, +0.39610653, -0.12867234, +0.00946086,
      -0.41463532, +0.88989262, -0.19018065, +0.16555497,
      +0.03917276, +0.22625372, +0.97328041, -0.02941475,
       0.00000000,  0.00000000,  0.00000000,  1.00000000});
    freeze_camera_pointcloud_enabled_ =
      this->declare_parameter<bool>("tzb.freeze_camera_pointcloud_enabled", true);
    publish_frozen_camera_tf_ =
      this->declare_parameter<bool>("tzb.publish_frozen_camera_tf", true);
    camera_pointcloud_topic_ = this->declare_parameter<std::string>(
      "tzb.camera_pointcloud_topic", "/camera/depth/color/points");
    frozen_camera_pointcloud_topic_ = this->declare_parameter<std::string>(
      "tzb.frozen_camera_pointcloud_topic", "/tzb_catch/frozen_observe_camera_points");
    frozen_camera_frame_id_ = this->declare_parameter<std::string>(
      "tzb.frozen_camera_frame_id", "tzb_observe_camera_link");
    apply_reference_transform_to_frozen_frame_ =
      this->declare_parameter<bool>("tzb.apply_reference_transform_to_frozen_frame", true);
    transform_frozen_pointcloud_to_reference_frame_ =
      this->declare_parameter<bool>("tzb.transform_frozen_pointcloud_to_reference_frame", true);
    // 冻结相机帧用"规划出的观测关节角"而不是"实测关节角"来算:
    // 实测角每次都差 0~0.1 rad, 会让同一个物体在 base 系里的坐标每次差几厘米。
    freeze_reference_from_planned_observe_ =
      this->declare_parameter<bool>("tzb.freeze_reference_from_planned_observe", true);
    // Hand-eye chain: the frozen camera reference is FK(camera_link) at the MEASURED
    // joints (median over the lock window), so the grasp target follows where the arm
    // actually stopped (observe J1 is typically -0.025 rad, not 0 -> ~1.2 cm lateral).
    // Overrides freeze_reference_from_planned_observe when true.
    camera_frame_from_measured_joints_ =
      this->declare_parameter<bool>("tzb.camera_frame_from_measured_joints", false);
    frozen_camera_pointcloud_log_throttle_sec_ = std::max(
      0.1,
      this->declare_parameter<double>("tzb.frozen_camera_pointcloud_log_throttle_sec", 2.0));
    pose_estimator_axis_conversion_ = declareMat3Param(
      "tzb.pose_estimator_axis_conversion_matrix",
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0});
    {
      const auto corr_rpy = this->declare_parameter<std::vector<double>>(
        "tzb.target_correction_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
      const auto corr_xyz = this->declare_parameter<std::vector<double>>(
        "tzb.target_correction_xyz", std::vector<double>{0.0, 0.0, 0.0});
      if (corr_rpy.size() == 3 && corr_xyz.size() == 3) {
        for (int i = 0; i < 3; ++i) {
          target_correction_rpy_deg_(i) = corr_rpy[i];
          target_correction_xyz_(i) = corr_xyz[i];
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
          "tzb.target_correction_rpy_deg / tzb.target_correction_xyz need 3 values; ignoring");
      }
      target_min_z_ = this->declare_parameter<double>("tzb.target_min_z", -1.0);
    target_left_y_threshold_ = this->declare_parameter<double>(
      "tzb.target_left_y_threshold", -1.0);
    target_left_y_offset_ = this->declare_parameter<double>(
      "tzb.target_left_y_offset", 0.0);
    // 只转姿态、不动位置的角度微调(deg, 绕 base_link z)。
    grasp_yaw_offset_deg_ = this->declare_parameter<double>(
      "tzb.grasp_yaw_offset_deg", 0.0);
    }
    gripper_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "tzb.gripper_joint_names",
      std::vector<std::string>{"Claw_left_joint"});
    gripper_home_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.gripper_home_positions", std::vector<double>{0.0});
    gripper_open_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.gripper_open_positions", std::vector<double>{-0.5236});
    gripper_close_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.gripper_close_positions", std::vector<double>{0.0});
    gripper_command_topic_ = this->declare_parameter<std::string>(
      "tzb.gripper_command_topic", "/tzb_catch/gripper_command");
    // Gripper serial protocol: -1 = open, 1/2/3 = closed mode 1/2/3.
    // Per-class override: tzb.class_profiles["<class>"]["gripper_close_level"].
    gripper_close_level_ = this->declare_parameter<int>("tzb.gripper_close_level", 1);
    if (gripper_close_level_ < 1 || gripper_close_level_ > 3) {
      throw std::runtime_error("tzb.gripper_close_level must be 1, 2 or 3");
    }
    grasp_collision_links_ = this->declare_parameter<std::vector<std::string>>(
      "tzb.grasp_collision_links",
      std::vector<std::string>{"Claw_left_Link", "claw_right_Link"});
    grasp_collision_allowed_penetration_m_ = this->declare_parameter<double>(
      "tzb.grasp_collision_allowed_penetration", -1.0);
    e3_collision_object_enabled_ = this->declare_parameter<bool>(
      "tzb.e3_collision_object_enabled", true);

    joint_planning_timeout_ = std::max(
      0.05,
      this->declare_parameter<double>("tzb.joint_planning_timeout", 2.0));
    cartesian_planning_timeout_ = std::max(
      0.05,
      this->declare_parameter<double>("tzb.cartesian_planning_timeout", 2.0));
    path_check_step_m_ = std::max(
      0.001,
      this->declare_parameter<double>("tzb.path_check_step", 0.05));
    cartesian_max_step_m_ = std::max(
      1e-4,
      this->declare_parameter<double>("tzb.cartesian_max_step_m", 0.02));
    cartesian_min_interpolation_steps_ = std::max<int>(
      1,
      this->declare_parameter<int>("tzb.cartesian_min_interpolation_steps", 2));
    cartesian_refinement_max_steps_ = std::max<int>(
      cartesian_min_interpolation_steps_,
      this->declare_parameter<int>("tzb.cartesian_refinement_max_steps", 16));
    cartesian_use_request_solver_settings_ =
      this->declare_parameter<bool>("tzb.cartesian_use_request_solver_settings", true);
    cartesian_use_sdls_ = this->declare_parameter<bool>("tzb.cartesian_use_sdls", true);
    cartesian_ik_max_iterations_ = std::max<int>(
      50,
      this->declare_parameter<int>("tzb.cartesian_ik_max_iterations", 800));
    cartesian_ik_pos_tolerance_ = std::max(
      1e-5,
      this->declare_parameter<double>("tzb.cartesian_ik_pos_tolerance", 0.002));
    cartesian_ik_rot_tolerance_ = std::max(
      1e-5,
      this->declare_parameter<double>("tzb.cartesian_ik_rot_tolerance", 0.05));
    cartesian_ik_damping_ = std::max(
      1e-12,
      this->declare_parameter<double>("tzb.cartesian_ik_damping", 1.0e-3));
    cartesian_ik_alpha_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.cartesian_ik_alpha", 0.55));
    cartesian_ik_max_step_norm_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.cartesian_ik_max_step_norm", 0.25));
    cartesian_ik_max_seed_count_ = std::max<int>(
      1,
      this->declare_parameter<int>("tzb.cartesian_ik_max_seed_count", 12));
    cartesian_nullspace_gain_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.cartesian_nullspace_gain", 0.0));
    cartesian_singular_threshold_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_singular_threshold", 0.03));
    cartesian_branch_jump_max_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_branch_jump_max_rad", 0.75));
    cartesian_branch_jump_norm_max_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_branch_jump_norm_max", 1.20));
    cartesian_waypoint_pos_tolerance_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_waypoint_pos_tolerance", 0.002));
    cartesian_waypoint_rot_tolerance_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_waypoint_rot_tolerance", 0.05));

    post_action_wait_sec_ = this->declare_parameter<double>("tzb.post_action_wait_sec", 0.05);
    // 夹爪只有 2 态指令(1=闭/-1=开)、串口协议里没有夹爪位置反馈,
    // 所以闭合指令之后必须等夹爪走完再抬臂, 否则会"一边合一边走"。
    gripper_settle_sec_ = std::max(
      0.0, this->declare_parameter<double>("tzb.gripper_settle_sec", 1.0));
    service_wait_sec_ = this->declare_parameter<double>("tzb.service_wait_sec", 30.0);
    const double configured_response_wait_sec =
      this->declare_parameter<double>("tzb.response_wait_sec", 30.0);
    response_wait_sec_ = std::max(
      configured_response_wait_sec,
      std::max(joint_planning_timeout_, cartesian_planning_timeout_) + 1.0);
    if (response_wait_sec_ > configured_response_wait_sec) {
      RCLCPP_WARN(
        this->get_logger(),
        "tzb.response_wait_sec %.2f is shorter than the planner timeout; using %.2f s.",
        configured_response_wait_sec,
        response_wait_sec_);
    }
    e3_marker_topic_ = this->declare_parameter<std::string>("tzb.e3_marker_topic", "/tzb_catch/e3_marker");
    publish_e3_marker_ = this->declare_parameter<bool>("tzb.publish_e3_marker", true);
    reference_marker_republish_hz_ = std::max(
      0.1,
      this->declare_parameter<double>("tzb.reference_marker_republish_hz", 5.0));
    publish_tcp_trajectory_ =
      this->declare_parameter<bool>("tzb.publish_tcp_trajectory", true);
    tcp_trajectory_line_width_m_ = std::max(
      1.0e-4,
      this->declare_parameter<double>("tzb.tcp_trajectory_line_width_m", 0.006));
    tcp_trajectory_min_distance_m_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.tcp_trajectory_min_distance_m", 0.002));
    tcp_trajectory_max_points_ = static_cast<size_t>(std::max<int64_t>(
      2,
      this->declare_parameter<int64_t>("tzb.tcp_trajectory_max_points", 10000)));
    sequence_loop_ = this->declare_parameter<bool>("tzb.sequence_loop", false);
    loop_pause_sec_ = std::max(0.0, this->declare_parameter<double>("tzb.loop_pause_sec", 0.5));
    replay_after_complete_ = this->declare_parameter<bool>("tzb.replay_after_complete", true);
    replay_loop_ = this->declare_parameter<bool>("tzb.replay_loop", true);
    replay_speed_scale_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.replay_speed_scale", 0.35));
    replay_segment_pause_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.replay_segment_pause_sec", 0.15));
    sentry_mode_ = this->declare_parameter<bool>("tzb.sentry_mode", false);
    sentry_execution_limit_ = std::max<int>(
      1, this->declare_parameter<int>("tzb.sentry_execution_limit", 1));
    arm2nav_state_topic_ = this->declare_parameter<std::string>(
      "tzb.arm2nav_state_topic", "/arm2nav");
    arm2nav_arm_state_topic_ = this->declare_parameter<std::string>(
      "tzb.arm2nav_arm_state_topic", "/arm2nav/from_arm");
    sentry_trigger_pulse_sec_ = std::max(
      0.1, this->declare_parameter<double>("tzb.sentry_trigger_pulse_sec", 0.2));
    joint_goal_tolerance_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.joint_goal_tolerance_rad", 0.01));
    arm_command_enabled_ = this->declare_parameter<bool>("tzb.arm_command_enabled", true);
    arm_command_publish_hz_ = std::max(
      1.0,
      this->declare_parameter<double>("tzb.arm_command_publish_hz", 100.0));
    arm_command_speed_scale_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.arm_command_speed_scale", 1.0));
    arm_command_goal_tolerance_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.arm_command_goal_tolerance_rad", 0.005));
    arm_command_publish_during_replay_ =
      this->declare_parameter<bool>("tzb.arm_command_publish_during_replay", false);

    enable_visualization_ = this->declare_parameter<bool>("enable_visualization", true);
    visualization_playback_enabled_ =
      enable_visualization_ && this->declare_parameter<bool>("enable_joint_state_playback", true);
    visualization_playback_speed_scale_ = std::max(
      1e-3,
      this->declare_parameter<double>("joint_state_playback_speed_scale", 1.0));

    robot_model_ = std::make_unique<RobotModel>(urdf_path, srdf_path, mesh_package_dirs);
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);

    joint_positions_unit_ =
      normalizeAngleUnit(this->declare_parameter<std::string>("tzb.joint_positions_unit", "rad"));
    if (!isRadianUnit(joint_positions_unit_) && !isDegreeUnit(joint_positions_unit_)) {
      throw std::runtime_error("tzb.joint_positions_unit must be 'rad' or 'deg'");
    }
    initial_arm_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.ini_joint_positions", std::vector<double>(arm_group.joint_names.size(), 0.0));
    pre_initial_arm_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.pre_ini_joint_positions", initial_arm_positions_);
    // Compact carry pose used between the wall-side grasp and the rear place.
    // The base joint is held while folding here, then rotated while the arm is
    // folded, so the 180 deg turn no longer sweeps at full extension.
    carry_tuck_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.carry_tuck_joint_positions",
      std::vector<double>{0.0, 0.7, -2.2, 0.0, 1.3, 0.0});
    staged_yaw_enabled_ = this->declare_parameter<bool>("tzb.staged_yaw_enabled", true);
    staged_yaw_min_rad_ = std::max(
      0.0, this->declare_parameter<double>("tzb.staged_yaw_min_rad", 0.5));
    place_arm_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.place_joint_positions", initial_arm_positions_);
    observe_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.observe_joint_positions",
      std::vector<double>{0.0, 0.0, 1.7, 0.0, 0.0, 0.0});
    sentry_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.sentry_joint_positions", observe_joint_positions_);
    max_arm_command_velocity_rad_s_ = this->declare_parameter<std::vector<double>>(
      "tzb.max_arm_command_velocity_rad_s",
      std::vector<double>{1.2, 1.2, 1.2, 1.8, 1.8, 2.2});
    arm_state_topic_ = this->declare_parameter<std::string>(
      "tzb.arm_state_topic", "/serial/arm_joint_state");
    arm_feedback_enabled_ = this->declare_parameter<bool>(
      "tzb.arm_feedback_enabled", true);
    arm_state_wait_timeout_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.arm_state_wait_timeout_sec", 2.0));
    arm_state_max_age_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.arm_state_max_age_sec", 0.5));
    arm_motion_feedback_timeout_sec_ = std::max(
      0.1,
      this->declare_parameter<double>("tzb.arm_motion_feedback_timeout_sec", 30.0));
    arm_motion_goal_tolerance_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.arm_motion_goal_tolerance_rad", 0.03));
    j6_arrival_tolerance_rad_ =
      this->declare_parameter<double>("tzb.j6_arrival_tolerance_rad", -1.0);
    arm_motion_stable_duration_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.arm_motion_stable_duration_sec", 0.25));
    require_arm_feedback_for_completion_ = this->declare_parameter<bool>(
      "tzb.require_arm_feedback_for_completion", true);
    // Precise pre-grasp arrival (2026-11-06 real grasps #5-#7): the normal arrival
    // accepts the grasp pose while J6 is still turning at ~1 rad/s (0.4 rad band,
    // 0.25 s window) and the lower controller settles 0.04-0.08 rad away from the
    // command (J1 friction band, J5 gravity sag) -> TCP 2-3 cm / 7-10 deg off at
    // the close. The grasp-approach stage therefore (1) waits until every joint
    // has stopped, (2) measures the residual and re-commands target + residual
    // (integral style, clamped), (3) closes the jaws only inside the tight band.
    grasp_precise_arrival_enabled_ =
      this->declare_parameter<bool>("tzb.grasp_precise_arrival_enabled", true);
    grasp_arrival_tolerance_rad_ = std::max(
      1e-4, this->declare_parameter<double>("tzb.grasp_arrival_tolerance_rad", 0.015));
    grasp_arrival_tolerance_j6_rad_ = std::max(
      1e-4, this->declare_parameter<double>("tzb.grasp_arrival_tolerance_j6_rad", 0.05));
    grasp_arrival_stable_sec_ = std::max(
      0.05, this->declare_parameter<double>("tzb.grasp_arrival_stable_sec", 0.5));
    grasp_arrival_max_speed_rad_s_ = std::max(
      1e-4, this->declare_parameter<double>("tzb.grasp_arrival_max_speed_rad_s", 0.02));
    grasp_arrival_timeout_sec_ = std::max(
      0.5, this->declare_parameter<double>("tzb.grasp_arrival_timeout_sec", 8.0));
    grasp_residual_gain_ = std::clamp(
      this->declare_parameter<double>("tzb.grasp_residual_gain", 1.0), 0.0, 2.0);
    grasp_residual_max_rad_ = std::max(
      0.0, this->declare_parameter<double>("tzb.grasp_residual_max_rad", 0.10));
    grasp_residual_iterations_ = static_cast<int>(std::max<int64_t>(
      0, this->declare_parameter<int>("tzb.grasp_residual_iterations", 3)));
    // Optional two-step approach: precise-arrive at a pose this far straight above
    // the grasp pose (same orientation), then descend. 0 = direct approach.
    grasp_contact_stop_enabled_ = this->declare_parameter<bool>("tzb.grasp_contact_stop_enabled", true);
    grasp_residual_lead_max_rad_ = this->declare_parameter<double>("tzb.grasp_residual_lead_max_rad", 0.03);
    grasp_contact_xy_tol_m_ = this->declare_parameter<double>("tzb.grasp_contact_xy_tol_m", 0.01);
    grasp_contact_above_m_ = this->declare_parameter<double>("tzb.grasp_contact_above_m", 0.02);
    grasp_contact_below_m_ = this->declare_parameter<double>("tzb.grasp_contact_below_m", 0.005);
    grasp_hover_height_m_ = std::max(
      0.0, this->declare_parameter<double>("tzb.grasp_hover_height_m", 0.0));
    // Early "nothing moves" warning while an arrival wait is still running
    // (remote controller off / arm not enabled / arm power); <= 0 disables it.
    no_motion_warning_sec_ =
      this->declare_parameter<double>("tzb.no_motion_warning_sec", 2.0);

    if (initial_arm_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.ini_joint_positions size does not match tzb arm group dof");
    }
    if (pre_initial_arm_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.pre_ini_joint_positions size does not match tzb arm group dof");
    }
    if (carry_tuck_joint_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.carry_tuck_joint_positions size does not match tzb arm group dof");
    }
    if (place_arm_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.place_joint_positions size does not match tzb arm group dof");
    }
    if (observe_joint_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.observe_joint_positions size does not match tzb arm group dof");
    }
    if (sentry_joint_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.sentry_joint_positions size does not match tzb arm group dof");
    }
    if (isDegreeUnit(joint_positions_unit_)) {
      degreesToRadiansInPlace(initial_arm_positions_);
      degreesToRadiansInPlace(pre_initial_arm_positions_);
      degreesToRadiansInPlace(carry_tuck_joint_positions_);
      degreesToRadiansInPlace(place_arm_positions_);
      degreesToRadiansInPlace(observe_joint_positions_);
      degreesToRadiansInPlace(sentry_joint_positions_);
    }
    if (max_arm_command_velocity_rad_s_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.max_arm_command_velocity_rad_s size does not match tzb arm group dof");
    }
    if (gripper_home_positions_.size() != gripper_joint_names_.size() ||
        gripper_open_positions_.size() != gripper_joint_names_.size() ||
        gripper_close_positions_.size() != gripper_joint_names_.size()) {
      throw std::runtime_error("tzb gripper position arrays must match tzb.gripper_joint_names size");
    }

    arm_state_ = initial_arm_positions_;
    gripper_state_ = gripper_open_positions_;

    task_autostart_ = this->declare_parameter<bool>("tzb.autostart", false);
    listen_navigation_goal_status_ = this->declare_parameter<bool>(
      "tzb.listen_navigation_goal_status", true);
    task_start_topic_ = this->declare_parameter<std::string>(
      "tzb.start_topic", "/tzb_catch/start");
    navigation_goal_status_topic_ = this->declare_parameter<std::string>(
      "tzb.navigation_goal_status_topic", "/navigation_goal_status");
    task_result_topic_ = this->declare_parameter<std::string>(
      "tzb.result_topic", "/tzb_catch/result");
    mission_controlled_ = this->declare_parameter<bool>("tzb.mission_controlled", false);
    // Graceful shutdown (issue 5): on the first SIGINT/SIGTERM the in-flight
    // operation is aborted at a safe point and the arm is homed to
    // pre_ini_joint_positions with physical-arrival confirmation.
    home_on_shutdown_ = this->declare_parameter<bool>("tzb.home_on_shutdown", true);
    shutdown_home_timeout_sec_ = std::max(
      1.0, this->declare_parameter<double>("tzb.shutdown_home_timeout_sec", 35.0));
    if (shutdown_home_timeout_sec_ > 40.0) {
      // The launch files escalate to SIGTERM at 55 s and SIGKILL at 65 s and
      // the start script waits timeout+20 s; a longer homing budget would be
      // cut short by SIGKILL mid-motion.
      RCLCPP_ERROR(
        this->get_logger(),
        "tzb.shutdown_home_timeout_sec=%.1f exceeds 40 s: launch sigterm_timeout (55 s) must stay >= timeout + 15 s; lower the timeout or raise the launch escalation timeouts.",
        shutdown_home_timeout_sec_);
    }
    // Policy when SIGINT/SIGTERM/SIGHUP arrives while the gripper holds the
    // object: "place" finishes place -> open -> pre_ini (the normal completion
    // steps); "hold" stops where the arm is and reports SKIPPED (exit 2).
    shutdown_with_object_ = this->declare_parameter<std::string>("tzb.shutdown_with_object", "place");
    if (shutdown_with_object_ != "place" && shutdown_with_object_ != "hold") {
      throw std::runtime_error("tzb.shutdown_with_object must be 'place' or 'hold'");
    }
    mission_feedback_grace_ = this->declare_parameter<double>("tzb.mission_feedback_grace_sec", 2.0);
    if(!std::isfinite(mission_feedback_grace_) || mission_feedback_grace_<=0)
      throw std::runtime_error("mission_feedback_grace_sec must be positive");
    navigation_sensor_frame_ = this->declare_parameter<std::string>(
      "tzb.navigation_sensor_frame", "camera_color_optical_frame");
    task_start_gate_enabled_ = !mission_controlled_ && !task_autostart_ && listen_navigation_goal_status_ && !sentry_mode_;
    if (mission_controlled_ && (!arm_feedback_enabled_ || !require_arm_feedback_for_completion_ ||
        sentry_mode_ || use_yaml_e3_target_ || replay_after_complete_ || sequence_loop_)) {
      throw std::runtime_error("Mission mode requires physical feedback, visual targets and non-looping, non-sentry execution");
    }

    plan_client_ = this->create_client<lite_motion_msgs::srv::PlanArmMotion>("plan_arm_motion");
    add_object_client_ = this->create_client<lite_motion_msgs::srv::AddSceneObject>("add_scene_object");
    attach_object_client_ = this->create_client<lite_motion_msgs::srv::AttachSceneObject>("attach_scene_object");
    remove_object_client_ = this->create_client<lite_motion_msgs::srv::RemoveSceneObject>("remove_scene_object");
    set_scene_collision_allowance_client_ =
      this->create_client<lite_motion_msgs::srv::SetSceneCollisionAllowance>(
        "set_scene_collision_allowance");
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->declare_parameter<std::string>(
        "joint_trajectory_topic", "/joint_trajectory_controller/joint_trajectory"),
      10);
    gripper_command_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      gripper_command_topic_, rclcpp::QoS(1).transient_local().reliable());
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      this->declare_parameter<std::string>("visualization_joint_states_topic", "/joint_states"),
      rclcpp::QoS(10));
    if (arm_command_enabled_) {
      arm_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        this->declare_parameter<std::string>(
          "tzb.arm_command_topic", "/tzb_catch/arm_joint_command"),
        rclcpp::SensorDataQoS());
    }
    if (arm_feedback_enabled_) {
      arm_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        arm_state_topic_,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          onArmState(*msg);
        });
    }
    if (task_start_gate_enabled_) {
      task_result_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        task_result_topic_, rclcpp::QoS(10).reliable());
      task_start_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        task_start_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
          if (msg->data != 1) {
            return;
          }
          {
            std::lock_guard<std::mutex> lock(task_trigger_mutex_);
            task_trigger_pending_ = true;
          }
          task_trigger_cv_.notify_all();
        });
      navigation_goal_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        navigation_goal_status_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
          if (msg->data == 0) {
            std::lock_guard<std::mutex> lock(task_trigger_mutex_);
            navigation_trigger_armed_ = true;
            return;
          }
          if (msg->data != 1) {
            RCLCPP_WARN(
              this->get_logger(),
              "Ignoring unsupported navigation goal status %d on [%s]; expected 0 or 1.",
              static_cast<int>(msg->data),
              navigation_goal_status_topic_.c_str());
            return;
          }
          bool should_notify = false;
          {
            std::lock_guard<std::mutex> lock(task_trigger_mutex_);
            if (navigation_trigger_armed_) {
              task_trigger_pending_ = true;
              navigation_trigger_armed_ = false;
              should_notify = true;
            }
          }
          if (should_notify) {
            RCLCPP_INFO(
              this->get_logger(),
              "Navigation goal status is 1 on [%s]; triggering the arm task.",
              navigation_goal_status_topic_.c_str());
            task_trigger_cv_.notify_all();
          }
        });
    }
    if (sentry_mode_) {
      arm2nav_state_sub_ = this->create_subscription<arm2nav_msgs::msg::Arm2Nav>(
        arm2nav_state_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const arm2nav_msgs::msg::Arm2Nav::SharedPtr msg) {
          onArm2NavState(*msg);
        });
      arm2nav_arm_state_pub_ = this->create_publisher<arm2nav_msgs::msg::Arm2Nav>(
        arm2nav_arm_state_topic_, rclcpp::QoS(10).reliable());
    }
    if (!use_yaml_e3_target_) {
      pose_estimator_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_estimator_target_topic_,
        rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onPoseEstimatorTarget(*msg);
        });
    }
    // TF 广播和点云转发解耦：只广播 TF 很便宜（10 Hz 一条变换），
    // 转发整幅点云很贵，所以允许只开 TF。
    if (freeze_camera_pointcloud_enabled_ || publish_frozen_camera_tf_) {
      frozen_camera_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      frozen_camera_tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          publishFrozenCameraFrame();
        });
    }
    if (freeze_camera_pointcloud_enabled_) {
      rclcpp::QoS frozen_cloud_qos(2);
      frozen_cloud_qos.reliable().durability_volatile();
      frozen_camera_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
          frozen_camera_pointcloud_topic_, frozen_cloud_qos);
      camera_pointcloud_sub_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
          camera_pointcloud_topic_,
          rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            onCameraPointCloud(*msg);
          });
    }

    if (enable_visualization_) {
      rclcpp::QoS latched_qos(1);
      latched_qos.transient_local().reliable();
      ref_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "tzb_catch/reference_frames", latched_qos);
      if (publish_e3_marker_) {
        e3_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
          e3_marker_topic_, latched_qos);
      }
      reference_marker_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / reference_marker_republish_hz_)),
        [this]() {
          publishReferenceMarkers();
          publishE3Marker();
        });
    }

    RCLCPP_INFO(
      this->get_logger(),
      "tzb_catch configured: arm_group=%s tcp_frame=%s target_source=%s continuous_required=%.2fs arm_state=%s initial_wait=%.2fs arrival_wait=%.2fs navigation_status_listener=%s sentry_mode=%s force_rpy=(roll:%s pitch:%s yaw:%s)",
      arm_group_.c_str(),
      tcp_frame_.c_str(),
      use_yaml_e3_target_ ? "yaml" : pose_estimator_target_topic_.c_str(),
      pose_estimator_continuous_required_sec_,
      arm_state_topic_.c_str(),
      arm_state_wait_timeout_sec_,
      arm_motion_feedback_timeout_sec_,
      task_start_gate_enabled_ ? "enabled" : "disabled",
      sentry_mode_ ? "enabled" : "disabled",
      force_target_roll_ ? "true" : "false",
      force_target_pitch_ ? "true" : "false",
      force_target_yaw_ ? "true" : "false");
    // ---- per-class grasp profiles (class comes from /mission/target_class) ----
    target_class_ = normalizeClassName(
      this->declare_parameter<std::string>("tzb.default_target_class", "blue_block"));
    try {
      const std::string profiles_raw =
        this->declare_parameter<std::string>("tzb.class_profiles", "{}");
      class_profiles_ = nlohmann::json::parse(profiles_raw);
      if (!class_profiles_.is_object()) {
        throw std::runtime_error("tzb.class_profiles must be a JSON object");
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
        "[GRASP][class] invalid tzb.class_profiles (%s); no per-class overrides", e.what());
      class_profiles_ = nlohmann::json::object();
    }
    base_grasp_ground_z_ = grasp_ground_z_;
    base_grasp_ground_z_max_deviation_ = grasp_ground_z_max_deviation_;
    base_target_min_z_ = target_min_z_;
    base_grasp_yaw_offset_deg_ = grasp_yaw_offset_deg_;
    base_target_yaw_reference_offset_deg_ = target_yaw_reference_offset_deg_;
    base_target_yaw_symmetry_deg_ = target_yaw_symmetry_deg_;
    base_place_arm_positions_ = place_arm_positions_;
    base_gripper_open_positions_ = gripper_open_positions_;
    base_gripper_close_positions_ = gripper_close_positions_;
    base_gripper_close_level_ = gripper_close_level_;
    base_e3_object_id_ = e3_object_id_;
    base_e3_mesh_resource_ = e3_mesh_resource_;
    base_mesh_scale_ = mesh_scale_;
    base_position_only_target_ = position_only_target_;
    RCLCPP_INFO(this->get_logger(), "[GRASP][class] default='%s' profiles=%s",
      target_class_.c_str(), class_profiles_.dump().c_str());
    logArmJointState("configured_ini_joint_positions", initial_arm_positions_);
    logArmJointState("configured_pre_ini_joint_positions", pre_initial_arm_positions_);
    logArmJointState("configured_place_joint_positions", place_arm_positions_);
    logArmJointState("configured_observe_joint_positions", observe_joint_positions_);

    if (mission_controlled_) {
      mission_state_pub_ = create_publisher<std_msgs::msg::String>("/arm_task/state", 10);
      // Single trigger path: the /arm_task/execute action. The legacy JSON
      // command topic only exists when explicitly re-enabled.
      legacy_json_command_ = this->declare_parameter<bool>("tzb.legacy_json_command", false);
      if (legacy_json_command_) {
        RCLCPP_WARN(this->get_logger(), "[MISSION] legacy /arm_task/command JSON trigger enabled (tzb.legacy_json_command=true)");
        mission_command_sub_ = create_subscription<std_msgs::msg::String>("/arm_task/command", 10,
          [this](std_msgs::msg::String::ConstSharedPtr m) { onMissionCommand(m->data); });
      }
      mission_action_server_ = rclcpp_action::create_server<ExecuteArmTask>(
        this, "/arm_task/execute",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteArmTask::Goal> goal) {
          return onMissionGoal(*goal);
        },
        [this](const std::shared_ptr<ExecuteArmTaskGoalHandle> handle) { return onMissionCancel(handle); },
        [this](const std::shared_ptr<ExecuteArmTaskGoalHandle> handle) { onMissionGoalAccepted(handle); });
      mission_feedback_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { publishMissionFeedback(); });
      RCLCPP_INFO(this->get_logger(), "[MISSION] /arm_task/execute action server ready (expected_center frame must be %s); legacy /arm_task/command %s",
        planning_frame_.c_str(), legacy_json_command_ ? "enabled" : "disabled");
      navigation_center_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/target_center", 10);
      navigation_hint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/target_hint", 10);
      navigation_center_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/perception/object_center", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { navigationTarget(*m, true); });
      navigation_hint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/perception/target_hint", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { navigationTarget(*m, false); });
      navigation_candidate_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/perception/candidate", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { navigationTarget(*m, false); });
      mission_state_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishMissionState(); });
    }

    // Class selected by pose_estimator / far_target_locator (latched); drives per-class profiles.
    target_class_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/target_class", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        const auto cls = normalizeClassName(m->data);
        if (cls.empty() || cls == target_class_) return;
        target_class_ = cls;
        RCLCPP_INFO(get_logger(), "[GRASP][class] target class <- '%s'", target_class_.c_str());
      });

    // Shutdown lease for planning_node / rm_serial_driver: true from node
    // start until the shutdown sequence has finished, then false.
    shutdown_hold_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/tzb_catch/shutdown_hold", rclcpp::QoS(10));
    shutdown_hold_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publishShutdownHold(shutdown_lease_active_.load()); });

    start_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        start_timer_->cancel();
        publishCurrentTheoreticalJointState("initial_home");
        publishReferenceMarkers("initial");
        publishE3Marker();
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (worker_spawn_closed_) {
          RCLCPP_WARN(this->get_logger(), "[SHUTDOWN] shutdown requested before the task worker started; not starting it.");
          return;
        }
        worker_ = std::thread([this]() { runSequence(); });
      });
  }

  ~TzbCatchNode() override {
    stop_requested_.store(true);
    notifyAllWaiters();
    if (shutdown_thread_.joinable()) {
      shutdown_thread_.join();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  // ---------------------------------------------------------------------
  // Graceful shutdown (issue 5). These entry points are called from the
  // signal watcher thread in main(); everything else stays on the worker /
  // executor threads. Kept separate so that an /arm_task/execute action
  // server can later reuse abortInFlightOperation() / currentPhase().
  // ---------------------------------------------------------------------

  static constexpr double kRepeatedSignalIgnoreSec = 2.0;
  static constexpr const char* kShutdownHint =
    "press Ctrl+C again after 2 s or Ctrl+\\ to stop the arm where it is";

  // First SIGINT/SIGTERM/SIGHUP: abort the in-flight operation and start the
  // shutdown thread (homing / place-and-release). A repeat within
  // kRepeatedSignalIgnoreSec (operator double-press, tty + launch delivery) is
  // ignored; any later repeat is the operator saying "stop": emergency hold.
  bool requestGracefulShutdown(int signum) {
    if (shutdown_requested_.exchange(true)) {
      const double since = secondsSinceShutdownRequest();
      if (since < kRepeatedSignalIgnoreSec) {
        RCLCPP_WARN(
          this->get_logger(),
          "[SHUTDOWN][SIGNAL][IGNORED] signal=%d repeated %.2fs after the first (< %.1fs); shutdown motion continues; %s",
          signum, since, kRepeatedSignalIgnoreSec, kShutdownHint);
        return false;
      }
      RCLCPP_ERROR(
        this->get_logger(),
        "[SHUTDOWN][SIGNAL][ESCALATED] signal=%d repeated %.2fs after the first: emergency hold requested by the operator",
        signum, since);
      requestEmergencyAbort(signum);
      return false;
    }
    shutdown_requested_ns_.store(steadyNowNs());
    RCLCPP_WARN(
      this->get_logger(), "[SHUTDOWN][BEGIN] signal=%d phase=%s hint=\"%s\"",
      signum, currentPhase().c_str(), kShutdownHint);
    beginShutdownCommon();
    return true;
  }

  // SIGQUIT (or an escalated repeat): stop streaming and hold at the MEASURED
  // pose, no homing, exit code 3.
  void requestEmergencyAbort(int signum) {
    emergency_abort_.store(true);
    RCLCPP_ERROR(
      this->get_logger(),
      "[SHUTDOWN][EMERGENCY] signal=%d phase=%s immediate abort: holding the arm at its measured pose, no homing",
      signum, currentPhase().c_str());
    if (shutdown_requested_.exchange(true)) {
      // Shutdown motion is in flight: abort it too.
      abortInFlightOperation();
      publishMeasuredHold("emergency_hold");
      return;
    }
    shutdown_requested_ns_.store(steadyNowNs());
    RCLCPP_ERROR(this->get_logger(), "[SHUTDOWN][BEGIN] signal=%d phase=%s", signum, currentPhase().c_str());
    beginShutdownCommon();
    publishMeasuredHold("emergency_hold");
  }

  // L1: the emergency hold is the fresh measured pose; the last streamed
  // sample is only used when feedback is stale. Nothing is sent if this
  // process never commanded the arm (H1).
  void publishMeasuredHold(const std::string& stage_name) {
    if (!arm_command_pub_ || !arm_command_sent_.load()) {
      return;
    }
    std::vector<double> hold;
    bool fresh = false;
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      fresh = have_complete_arm_state_ &&
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - latest_arm_state_time_).count() <= arm_state_max_age_sec_;
      if (fresh) {
        hold = latest_measured_arm_positions_;
      }
    }
    if (!fresh) {
      RCLCPP_WARN(this->get_logger(), "[SHUTDOWN][HOLD] %s: feedback stale, keeping the last streamed sample", stage_name.c_str());
      return;
    }
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    publishArmCommand(group.joint_names, hold);
    applyNamedJointPositions(group.joint_names, hold);
    logArmJointState(stage_name + "_measured", hold);
  }

  bool shutdownRequested() const { return shutdown_requested_.load(); }
  bool shutdownFinished() const { return shutdown_finished_.load(); }
  int shutdownExitCode() const { return shutdown_exit_code_.load(); }
  double shutdownHomeTimeoutSec() const { return shutdown_home_timeout_sec_; }

  double secondsSinceShutdownRequest() const {
    const int64_t requested_ns = shutdown_requested_ns_.load();
    if (requested_ns == 0) {
      return 0.0;
    }
    return static_cast<double>(steadyNowNs() - requested_ns) * 1e-9;
  }

  static int64_t steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void joinShutdownThread() {
    if (shutdown_thread_.joinable()) {
      shutdown_thread_.join();
    }
  }

  void publishShutdownHold(bool hold) {
    if (!shutdown_hold_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = hold;
    shutdown_hold_pub_->publish(msg);
  }

private:
  // Either the destructor's stop, the shutdown abort of the current operation,
  // or an action cancel abort of the current mission operation (cleared by the
  // mission worker before it recovers to pre_ini).
  bool stopping() const { return stop_requested_.load() || operation_abort_.load() || mission_abort_.load(); }

  void notifyAllWaiters() {
    pose_estimator_cv_.notify_all();
    arm_state_cv_.notify_all();
    task_trigger_cv_.notify_all();
    sentry_cv_.notify_all();
    mission_cv_.notify_all();
  }

  // Make every blocking wait of the worker return promptly. Trajectory
  // streaming holds the last streamed command (see publishArmCommandTrajectory).
  void abortInFlightOperation() {
    operation_abort_.store(true);
    notifyAllWaiters();
  }

  void beginShutdownCommon() {
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      mission_phase_ = "SHUTDOWN";
      mission_pending_ = false;
    }
    abortInFlightOperation();
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_spawn_closed_ = true;
    shutdown_thread_ = std::thread([this]() { runShutdownSequence(); });
  }

  void setPhase(const std::string& phase) {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    current_phase_ = phase;
  }

  std::string currentPhase() const {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    return current_phase_;
  }

  void runShutdownSequence() {
    const std::chrono::steady_clock::time_point started{
      std::chrono::nanoseconds(shutdown_requested_ns_.load())};
    // 1. Let the task worker leave its current operation at a safe point.
    {
      std::thread worker;
      {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        worker.swap(worker_);
      }
      if (worker.joinable()) {
        worker.join();
      }
    }
    RCLCPP_INFO(
      this->get_logger(), "[SHUTDOWN] task worker stopped after %.2fs; phase=%s",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
      currentPhase().c_str());
    // 2. Home the arm (unless this is an emergency abort).
    int exit_code = 0;
    if (emergency_abort_.load()) {
      RCLCPP_ERROR(this->get_logger(), "[SHUTDOWN][HOME][FAIL] reason=emergency_abort (arm held at its measured pose)");
      exit_code = 3;
    } else {
      exit_code = runShutdownHoming(started) ? 0 : 2;
      if (emergency_abort_.load()) {
        exit_code = 3;
      }
    }
    // 3. A goal that was active or pending gets its terminal result now,
    //    reflecting the homing outcome (mission_node faults only on ERROR).
    abortMissionGoalOnShutdown(exit_code == 0);
    // 4. Release dependents and report.
    setPhase("waiting");
    shutdown_exit_code_.store(exit_code);
    shutdown_lease_active_.store(false);
    shutdown_finished_.store(true);
    publishShutdownHold(false);
    if (exit_code == 0) {
      RCLCPP_INFO(this->get_logger(), "[SHUTDOWN][END] exit_code=%d", exit_code);
    } else {
      RCLCPP_ERROR(this->get_logger(), "[SHUTDOWN][END] exit_code=%d", exit_code);
    }
    shutdown_cv_.notify_all();
  }

  // Returns true only when the arm is confirmed at pre_ini_joint_positions by
  // measured joint feedback, or when no motion is owed (logged as SKIPPED):
  // homing configured off, arm commands/feedback disabled, or this process
  // never commanded the arm (H1). With an object in the jaws the normal
  // completion steps (place -> release -> pre_ini) are run, or, with
  // tzb.shutdown_with_object=hold, the arm holds and SKIPPED is returned as a
  // failure (H3).
  bool runShutdownHoming(const std::chrono::steady_clock::time_point& started) {
    setPhase("home");
    if (!home_on_shutdown_) {
      RCLCPP_WARN(this->get_logger(), "[SHUTDOWN][HOME][SKIPPED] reason=home_on_shutdown=false");
      return true;
    }
    if (!arm_command_pub_) {
      RCLCPP_WARN(this->get_logger(), "[SHUTDOWN][HOME][SKIPPED] reason=arm_command_disabled");
      return true;
    }
    if (!arm_feedback_enabled_) {
      // Visualization-only configuration (use_serial_driver:=false): there is
      // no physical arm in the loop, so there is nothing to confirm.
      RCLCPP_WARN(this->get_logger(), "[SHUTDOWN][HOME][SKIPPED] reason=arm_feedback_disabled (no physical arm in the loop)");
      return true;
    }
    if (!arm_command_sent_.load()) {
      // The serial driver latches arm_command_enabled on the first command;
      // a stop key must never be the session's first arm motion.
      RCLCPP_WARN(this->get_logger(), "[SHUTDOWN][HOME][SKIPPED] reason=never_commanded (this process sent no arm command; arm left as is)");
      return true;
    }
    const bool object_held = gripper_closed_on_object_.load();
    if (object_held && shutdown_with_object_ == "hold") {
      publishMeasuredHold("shutdown_object_hold");
      RCLCPP_ERROR(this->get_logger(), "[SHUTDOWN][HOME][SKIPPED] reason=object_in_gripper (tzb.shutdown_with_object=hold; arm held at its measured pose, object still in the jaws)");
      return false;
    }
    // The homing gets its own abort budget: the deadline thread below (or a
    // SIGQUIT / escalated repeat) re-raises operation_abort_.
    operation_abort_.store(emergency_abort_.load());
    // Lease for planning_node / serial driver: true only while this motion runs.
    shutdown_lease_active_.store(true);
    publishShutdownHold(true);
    restricted_recovery_.store(true);
    const auto deadline =
      started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(shutdown_home_timeout_sec_));
    std::atomic_bool homing_done{false};
    std::atomic_bool deadline_hit{false};
    std::thread deadline_thread([this, &homing_done, &deadline_hit, deadline]() {
      std::unique_lock<std::mutex> lock(shutdown_cv_mutex_);
      const bool done = shutdown_cv_.wait_until(
        lock, deadline, [&homing_done]() { return homing_done.load(); });
      if (!done) {
        deadline_hit.store(true);
        RCLCPP_ERROR(
          this->get_logger(),
          "[SHUTDOWN][HOME][TIMEOUT] shutdown_home_timeout_sec=%.1f elapsed; aborting the shutdown motion and holding the last streamed command",
          shutdown_home_timeout_sec_);
        abortInFlightOperation();
      }
    });

    bool ok = false;
    std::string reason;
    last_stuck_joints_.clear();
    last_deadband_joints_.clear();
    last_feedback_stale_ = false;
    last_no_motion_ = false;
    last_plan_failure_.clear();
    last_service_timeout_ = false;
    try {
      // L5: a dead planner must fail at once, not after response_wait_sec.
      if (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
        reason = "planner_unavailable";
      } else if (!readInitialArmJointPositions(arm_state_)) {
        reason = "no_arm_feedback";
      } else if (object_held) {
        RCLCPP_WARN(this->get_logger(), "[SHUTDOWN][HOME] object in the gripper (tzb.shutdown_with_object=place): finishing place -> release -> pre_ini");
        if (!completePlaceAndReturn("shutdown_")) {
          reason = recoveryFailureReason();
        } else {
          ok = true;
        }
      } else if (!executeArmStagedYawPlan(pre_initial_arm_positions_, "shutdown_home")) {
        // M6: a pose already inside the tolerance still goes through the
        // plan/command/stability-window path above; no single-sample "already home".
        reason = recoveryFailureReason();
      } else {
        ok = true;
      }
    } catch (const std::exception& e) {
      reason = std::string("exception:") + e.what();
    }
    restricted_recovery_.store(false);

    // Confirm from measured feedback regardless of how the stage reported.
    double max_error = std::numeric_limits<double>::infinity();
    double normalized_error = std::numeric_limits<double>::infinity();
    std::string joint_errors;
    bool feedback_fresh = false;
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      feedback_fresh = have_complete_arm_state_ &&
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - latest_arm_state_time_).count() <= arm_state_max_age_sec_;
      if (have_complete_arm_state_) {
        max_error = maxJointError(latest_measured_arm_positions_, pre_initial_arm_positions_);
        normalized_error = maxNormalizedJointError(latest_measured_arm_positions_, pre_initial_arm_positions_);
        joint_errors = formatArmJointErrors(latest_measured_arm_positions_, pre_initial_arm_positions_);
      }
    }
    if (ok && !feedback_fresh) {
      ok = false;
      reason = "feedback_stale";
    } else if (ok && normalized_error > 1.0) {
      ok = false;
      reason = "not_confirmed_by_feedback";
    }
    {
      std::lock_guard<std::mutex> lock(shutdown_cv_mutex_);
      homing_done.store(true);
    }
    shutdown_cv_.notify_all();
    deadline_thread.join();
    if (!ok && deadline_hit.load()) {
      reason = "timeout(" + reason + ")";
    }
    if (emergency_abort_.load()) {
      ok = false;
      reason = "emergency_abort";
    }
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (ok) {
      RCLCPP_INFO(
        this->get_logger(),
        "[SHUTDOWN][HOME][OK] max_error=%.5f object_placed=%s elapsed=%.2fs",
        max_error, object_held ? "true" : "false", elapsed);
      return true;
    }
    RCLCPP_ERROR(
      this->get_logger(),
      "[SHUTDOWN][HOME][FAIL] reason=%s max_error=%.5f object_in_gripper=%s elapsed=%.2fs joint_errors:%s",
      reason.c_str(), max_error, gripper_closed_on_object_.load() ? "true" : "false", elapsed, joint_errors.c_str());
    return false;
  }

  static std::string normalizeClassName(const std::string &in) {
    std::string out;
    out.reserve(in.size());
    bool pending_space = false;
    for (char ch : in) {
      const unsigned char c = static_cast<unsigned char>(ch);
      if (c == '_' || c == '-' || std::isspace(c)) {
        if (!out.empty()) pending_space = true;
        continue;
      }
      if (pending_space) { out.push_back(' '); pending_space = false; }
      out.push_back(static_cast<char>(std::tolower(c)));
    }
    return out;
  }

  // Apply the per-class grasp profile to the live members (base values first, then overrides).
  void applyTargetClassProfile() {
    grasp_ground_z_ = base_grasp_ground_z_;
    grasp_ground_z_max_deviation_ = base_grasp_ground_z_max_deviation_;
    target_min_z_ = base_target_min_z_;
    grasp_yaw_offset_deg_ = base_grasp_yaw_offset_deg_;
    target_yaw_reference_offset_deg_ = base_target_yaw_reference_offset_deg_;
    target_yaw_symmetry_deg_ = base_target_yaw_symmetry_deg_;
    place_arm_positions_ = base_place_arm_positions_;
    gripper_open_positions_ = base_gripper_open_positions_;
    gripper_close_positions_ = base_gripper_close_positions_;
    gripper_close_level_ = base_gripper_close_level_;
    e3_object_id_ = base_e3_object_id_;
    e3_mesh_resource_ = base_e3_mesh_resource_;
    mesh_scale_ = base_mesh_scale_;
    position_only_target_ = base_position_only_target_;
    if (!class_profiles_.contains(target_class_)) {
      RCLCPP_INFO(this->get_logger(), "[GRASP][class] '%s' has no profile; using base params",
        target_class_.c_str());
      return;
    }
    const auto &pr = class_profiles_.at(target_class_);
    auto num = [&pr](const char *key, double &dst) {
      const auto it = pr.find(key);
      if (it != pr.end() && it->is_number()) dst = it->get<double>();
    };
    auto vec = [&pr](const char *key, std::vector<double> &dst) {
      const auto it = pr.find(key);
      if (it != pr.end() && it->is_array()) dst = it->get<std::vector<double>>();
    };
    auto str = [&pr](const char *key, std::string &dst) {
      const auto it = pr.find(key);
      if (it != pr.end() && it->is_string()) dst = it->get<std::string>();
    };
    auto boolean = [&pr](const char *key, bool &dst) {
      const auto it = pr.find(key);
      if (it != pr.end() && it->is_boolean()) dst = it->get<bool>();
    };
    auto integer = [&pr](const char *key, int &dst) {
      const auto it = pr.find(key);
      if (it != pr.end() && it->is_number_integer()) dst = it->get<int>();
    };
    num("grasp_ground_z", grasp_ground_z_);
    num("grasp_ground_z_max_deviation", grasp_ground_z_max_deviation_);
    num("target_min_z", target_min_z_);
    num("grasp_yaw_offset_deg", grasp_yaw_offset_deg_);
    num("target_yaw_reference_offset_deg", target_yaw_reference_offset_deg_);
    num("target_yaw_symmetry_deg", target_yaw_symmetry_deg_);
    boolean("position_only_target", position_only_target_);
    vec("place_arm_positions", place_arm_positions_);
    vec("gripper_open_positions", gripper_open_positions_);
    vec("gripper_close_positions", gripper_close_positions_);
    integer("gripper_close_level", gripper_close_level_);
    if (gripper_close_level_ < 1 || gripper_close_level_ > 3) {
      RCLCPP_WARN(this->get_logger(),
        "[GRASP][class] '%s' gripper_close_level out of range (%d); using 1",
        target_class_.c_str(), gripper_close_level_);
      gripper_close_level_ = 1;
    }
    vec("mesh_scale", mesh_scale_);
    str("e3_object_id", e3_object_id_);
    str("e3_mesh_resource", e3_mesh_resource_);
    RCLCPP_INFO(this->get_logger(),
      "[GRASP][class] applied '%s' profile: grasp_ground_z=%.4f pos_only=%d place_joints=%zu close_level=%d",
      target_class_.c_str(), grasp_ground_z_, static_cast<int>(position_only_target_),
      place_arm_positions_.size(), gripper_close_level_);
  }

  // Acceptance rules shared by the action server and the legacy JSON path
  // (contract C2 minus the stamp window). Returns "" when the task was claimed,
  // otherwise the rejection reason. Caller holds mission_mutex_.
  std::string claimMissionTaskLocked(
    const std::string& session, uint64_t id, const std::string& op, const std::vector<double>& expected) {
    if (shutdown_requested_.load()) return "shutdown in progress";
    if (mission_phase_ == "BUSY" || mission_pending_) return "a task is already executing";
    if (session != mission_session_ && op != "prepare") return "grasp from a session that has not prepared";
    if (session == mission_session_ && id <= mission_task_id_)
      return "task_id " + std::to_string(id) + " is not greater than the last task_id " + std::to_string(mission_task_id_) + " of this session";
    if (op != "prepare" && mission_phase_ != "READY") return op + " while phase is " + mission_phase_ + " (expected READY)";
    mission_session_ = session; mission_task_id_ = id; mission_operation_ = op;
    mission_expected_center_ = expected; mission_pending_ = true; mission_success_ = false; mission_phase_ = "BUSY";
    mission_cancel_requested_ = false;
    mission_goal_handle_.reset();
    return "";
  }

  // Legacy JSON trigger (tzb.legacy_json_command=true only). Keeps the 0.5 s
  // stamp window of the re-published command stream.
  void onMissionCommand(const std::string &data) {
    try {
      const auto j = nlohmann::json::parse(data);
      const auto session = j.at("session").get<std::string>();
      const auto id = j.at("task_id").get<uint64_t>();
      const auto op = j.at("command").get<std::string>();
      const double age = now().seconds()-j.at("stamp").get<double>();
      if (session.empty() || !std::isfinite(age) || age < -0.15 || age > 0.5 ||
          (op != "prepare" && op != "grasp")) return;
      std::vector<double> expected;
      if (op == "grasp") {
        expected = j.at("expected_center").get<std::vector<double>>();
        if (expected.size()!=3 || !std::all_of(expected.begin(),expected.end(),[](double v) { return std::isfinite(v); })) return;
      }
      std::lock_guard<std::mutex> lock(mission_mutex_);
      if (!claimMissionTaskLocked(session, id, op, expected).empty()) return;
      mission_awaiting_handle_ = false;
      mission_cv_.notify_all();
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Invalid mission arm command: %s",e.what());
    }
  }

  // ---- /arm_task/execute action server (contract G) ----------------------

  rclcpp_action::GoalResponse onMissionGoal(const ExecuteArmTask::Goal& goal) {
    std::string reason;
    std::vector<double> expected;
    const std::string op = calibBaseOp(goal.command);
    double calib_yaw_deg = 0.0;
    double calib_observe_j1 = 0.0;
    std::vector<double> calib_joints;
    std::string calib_joints_error;
    if (op == "calib_place" || op == "calib_regrasp" || op == "calib_check") {
      calib_yaw_deg = std::numeric_limits<double>::quiet_NaN();
      const auto k = goal.command.find(':');
      if (k != std::string::npos) {
        try { calib_yaw_deg = std::stod(goal.command.substr(k + 1)); } catch (const std::exception&) {}
      }
    } else if (op == "calib_observe") {
      // calib_observe[:<j1_rad>]: observe pose with J1 replaced (other joints unchanged)
      const auto k = goal.command.find(':');
      if (k != std::string::npos) {
        calib_observe_j1 = std::numeric_limits<double>::quiet_NaN();
        try { calib_observe_j1 = std::stod(goal.command.substr(k + 1)); } catch (const std::exception&) {}
      }
    } else if (op == "calib_joints") {
      calib_joints_error = parseCalibJointsCommand(goal.command, calib_joints);
    }
    if (goal.session.empty()) {
      reason = "empty session";
    } else if (op != "prepare" && op != "grasp" && op != "observe" && !isCalibOp(op)) {
      reason = "unknown command '" + goal.command + "'";
    } else if ((op == "calib_place" || op == "calib_regrasp" || op == "calib_check") && !std::isfinite(calib_yaw_deg)) {
      reason = "command '" + goal.command + "' needs the form " + op + ":<tool_yaw_deg>";
    } else if (op == "calib_observe" && !(std::isfinite(calib_observe_j1) && std::abs(calib_observe_j1) <= 1.0)) {
      reason = "command '" + goal.command + "' needs the form calib_observe[:<j1_rad>] with |j1| <= 1.0";
    } else if (op == "calib_joints" && !calib_joints_error.empty()) {
      reason = "command '" + goal.command + "' rejected: " + calib_joints_error;
    } else if (op != "prepare") {
      const auto& p = goal.expected_center.point;
      if (goal.expected_center.header.frame_id != planning_frame_) {
        // No silent relabelling: the identity gate compares this point with
        // /mission/target_center, which is expressed in planning_frame_.
        reason = "expected_center frame '" + goal.expected_center.header.frame_id +
          "' is not the arm planning frame '" + planning_frame_ + "'";
      } else if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        reason = "expected_center is not finite";
      } else {
        expected = {p.x, p.y, p.z};
      }
    }
    if (reason.empty()) {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      reason = claimMissionTaskLocked(goal.session, goal.task_id, op, expected);
      if (reason.empty()) {
        mission_calib_yaw_deg_ = calib_yaw_deg;
        mission_calib_observe_j1_ = calib_observe_j1;
        mission_calib_joints_ = calib_joints;
        mission_result_json_.clear();
        // The worker starts only once onMissionGoalAccepted has stored the handle.
        mission_awaiting_handle_ = true;
      }
    }
    if (!reason.empty()) {
      RCLCPP_WARN(get_logger(), "[MISSION][goal][REJECT] session=%s task_id=%llu command=%s reason=%s",
        goal.session.c_str(), static_cast<unsigned long long>(goal.task_id), goal.command.c_str(), reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "[MISSION][goal][ACCEPT] session=%s task_id=%llu command=%s expected_center=(%.4f, %.4f, %.4f)",
      goal.session.c_str(), static_cast<unsigned long long>(goal.task_id), goal.command.c_str(),
      expected.size() == 3 ? expected[0] : 0.0, expected.size() == 3 ? expected[1] : 0.0, expected.size() == 3 ? expected[2] : 0.0);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void onMissionGoalAccepted(const std::shared_ptr<ExecuteArmTaskGoalHandle> handle) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    mission_goal_handle_ = handle;
    mission_awaiting_handle_ = false;
    mission_cv_.notify_all();
  }

  // Cancel = finish the current segment at a hold (issue-5 abort mechanism),
  // then recover to pre_ini with physical confirmation; the terminal result
  // follows. A cancel during recovery just lets the recovery finish.
  rclcpp_action::CancelResponse onMissionCancel(const std::shared_ptr<ExecuteArmTaskGoalHandle> handle) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    if (!mission_goal_handle_ || handle != mission_goal_handle_) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    mission_cancel_requested_ = true;
    RCLCPP_WARN(get_logger(), "[MISSION][cancel] task_id=%llu operation=%s arm_phase=%s executing=%s: %s",
      static_cast<unsigned long long>(mission_task_id_), mission_operation_.c_str(), currentPhase().c_str(),
      mission_executing_ ? "true" : "false",
      mission_executing_ ? "holding the current segment, then recovering to pre_ini" : "will recover to pre_ini");
    if (mission_executing_) {
      mission_abort_.store(true);
      notifyAllWaiters();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Measured arm at pre_ini with fresh feedback: the only source of
  // safe_to_drive (heartbeat, feedback and result).
  bool measuredAtReadyPose() {
    const auto &ready_positions=pre_initial_arm_positions_;
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    if(!(have_complete_arm_state_ && latest_measured_arm_positions_.size()==ready_positions.size() &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-latest_arm_state_time_).count()<=std::max(arm_state_max_age_sec_,mission_feedback_grace_)))
      return false;
    // Same per-joint tolerances and equivalent-angle handling as
    // waitForArmAtTarget; in particular, J6 has its own calibrated tolerance.
    return maxNormalizedJointError(latest_measured_arm_positions_,ready_positions)<=1.0;
  }

  void publishMissionFeedback() {
    std::shared_ptr<ExecuteArmTaskGoalHandle> handle;
    std::string phase;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      handle = mission_goal_handle_;
      phase = mission_phase_;
    }
    if (!handle || !handle->is_active()) return;
    auto feedback = std::make_shared<ExecuteArmTask::Feedback>();
    feedback->phase = phase;
    feedback->safe_to_drive = measuredAtReadyPose();
    feedback->detail = currentPhase();
    try { handle->publish_feedback(feedback); } catch (const std::exception&) {}
  }

  // Heartbeat state first, then the terminal result (contract H4): a READY
  // heartbeat must never precede its own task's phase transition.
  void finishMissionTask(bool success, bool recovered) {
    std::shared_ptr<ExecuteArmTaskGoalHandle> handle;
    std::string phase, operation;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      mission_success_ = success;
      // An aborted operation must not overwrite the SHUTDOWN heartbeat phase;
      // the shutdown sequence sends the terminal result after homing.
      if (shutdown_requested_.load()) return;
      mission_phase_ = (success || recovered) ? "READY" : "ERROR";
      phase = mission_phase_;
      operation = mission_operation_;
      handle = std::move(mission_goal_handle_);
      mission_goal_handle_.reset();
      mission_cancel_requested_ = false;
    }
    if (!handle) return;  // legacy JSON task
    auto result = std::make_shared<ExecuteArmTask::Result>();
    result->success = success;
    result->safe_to_drive = measuredAtReadyPose();
    result->phase = phase;
    const bool canceling = handle->is_canceling();
    if (success) result->message = operation + (isCalibOp(operation) ? " completed" : " completed; arm at pre_ini");
    else if (canceling) result->message = operation + (recovered ? " canceled; recovered to pre_ini" : " canceled; recovery to pre_ini FAILED");
    else if (operation == "prepare") result->message = "prepare failed; arm not confirmed at pre_ini";  // no recovery stage after prepare
    else result->message = operation + (recovered ? " failed; recovered to pre_ini" : " failed and recovery to pre_ini FAILED");
    if (!success && last_no_motion_) {
      // Operator-facing: mission_node copies this message into its FAULT reason.
      result->message += std::string(" ") + kNoMotionReason +
        ": lower controller did not execute any joint command; " + kNoMotionAdvice;
    } else if (!last_stuck_joints_.empty()) result->message += " stuck_joint(" + last_stuck_joints_ + ")";
    else if (!last_grasp_arrival_failure_.empty() && !success) result->message += " grasp_arrival_failed(" + last_grasp_arrival_failure_ + ")";
    else if (!last_plan_failure_.empty() && !success) result->message += " plan_failed(" + last_plan_failure_ + ")";
    if (isCalibOp(operation) && !mission_result_json_.empty()) result->message += " json=" + mission_result_json_;
    try {
      if (canceling) handle->canceled(result);
      else if (success) handle->succeed(result);
      else handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "[MISSION][result] could not send the terminal result: %s", e.what());
    }
    RCLCPP_INFO(get_logger(), "[MISSION][result] task_id=%llu %s success=%s safe_to_drive=%s phase=%s message=%s",
      static_cast<unsigned long long>(mission_task_id_), canceling ? "canceled" : (success ? "succeeded" : "aborted"),
      success ? "true" : "false", result->safe_to_drive ? "true" : "false", phase.c_str(), result->message.c_str());
  }

  // Called by the shutdown sequence after homing: the active/pending goal
  // ends with an abort whose phase reflects the homing outcome.
  void abortMissionGoalOnShutdown(bool homed) {
    std::shared_ptr<ExecuteArmTaskGoalHandle> handle;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      handle = std::move(mission_goal_handle_);
      mission_goal_handle_.reset();
      mission_cancel_requested_ = false;
    }
    if (!handle) return;
    auto result = std::make_shared<ExecuteArmTask::Result>();
    result->success = false;
    result->safe_to_drive = measuredAtReadyPose();
    result->phase = homed ? "READY" : "ERROR";
    result->message = homed ? "tzb_catch shutting down; arm homed to pre_ini"
                            : "tzb_catch shutting down; shutdown homing FAILED, arm not confirmed at pre_ini";
    try {
      if (handle->is_canceling()) handle->canceled(result);
      else handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "[MISSION][result] could not send the shutdown result: %s", e.what());
    }
    RCLCPP_WARN(get_logger(), "[MISSION][result] task_id=%llu aborted by shutdown phase=%s safe_to_drive=%s",
      static_cast<unsigned long long>(mission_task_id_), result->phase.c_str(), result->safe_to_drive ? "true" : "false");
  }

  void publishMissionState() {
    nlohmann::json j;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      j={{"session",mission_session_},{"task_id",mission_task_id_},{"phase",mission_phase_},
        {"success",mission_success_},{"stamp",now().seconds()}};
    }
    j["arm_phase"]=currentPhase();
    // During SHUTDOWN the heartbeat keeps reporting whether the measured arm is
    // at the ready pose, so a supervisor can see the homing complete.
    const bool safe=(j["phase"]=="READY" || j["phase"]=="SHUTDOWN") && measuredAtReadyPose();
    j["safe_to_drive"]=safe;
    j["ready_pose"]="pre_ini";
    std_msgs::msg::String m; m.data=j.dump(); mission_state_pub_->publish(m);
  }

  // Cancel / failed-grasp recovery. No motion if this process never commanded
  // the arm (H1); an object in the jaws is placed and released first (H3);
  // "already at pre_ini" is never a single-sample judgement: pre_ini is
  // commanded and confirmed with the stability window (M6); no direct-sweep
  // fallback (M1).
  bool recoverToPreIni(const std::string& stage_name) {
    const std::string precheck = recoveryPrecheck();
    if (precheck == "never_commanded") {
      const bool at_ready = measuredAtReadyPose();
      RCLCPP_WARN(get_logger(), "[MISSION][%s] this process never commanded the arm; no motion (measured at pre_ini: %s)",
        stage_name.c_str(), at_ready ? "true" : "false");
      return at_ready;
    }
    if (!precheck.empty()) {
      RCLCPP_ERROR(get_logger(), "[MISSION][%s] recovery not attempted: reason=%s", stage_name.c_str(), precheck.c_str());
      return false;
    }
    restricted_recovery_.store(true);
    last_service_timeout_ = false;
    bool ok = false;
    std::string reason_override;
    try {
      if (gripper_closed_on_object_.load()) {
        if (shutdown_with_object_ == "hold") {
          RCLCPP_ERROR(get_logger(), "[MISSION][%s] object in the gripper and tzb.shutdown_with_object=hold: holding at the measured pose, no recovery",
            stage_name.c_str());
          publishMeasuredHold(stage_name + "_object_hold");
          reason_override = "object_in_gripper(policy=hold)";
          ok = false;
        } else {
          RCLCPP_WARN(get_logger(), "[MISSION][%s] object in the gripper: finishing place -> release -> pre_ini before reporting",
            stage_name.c_str());
          ok = completePlaceAndReturn(stage_name + "_");
        }
      } else {
        ok = executeArmStagedYawPlan(pre_initial_arm_positions_, stage_name);
      }
    } catch (...) {
      restricted_recovery_.store(false);
      throw;
    }
    restricted_recovery_.store(false);
    if (!ok) {
      if (!reason_override.empty()) last_plan_failure_ = reason_override;
      RCLCPP_ERROR(get_logger(), "[MISSION][%s] recovery FAILED reason=%s", stage_name.c_str(),
        reason_override.empty() ? recoveryFailureReason().c_str() : reason_override.c_str());
    }
    return ok;
  }

  void runMissionSequence() {
    while (!stopping()) {
      std::string operation;
      bool canceled_before_start=false;
      {
        std::unique_lock<std::mutex> lock(mission_mutex_);
        setPhase("waiting");
        mission_cv_.wait(lock,[this]() { return stopping() || (mission_pending_ && !mission_awaiting_handle_); });
        if(stopping()) return;
        mission_pending_=false; operation=mission_operation_;
        canceled_before_start=mission_cancel_requested_;
        mission_executing_=!canceled_before_start;
      }
      setPhase("prepare");
      bool success=false;
      if(!canceled_before_start) try {
        const bool services=waitForService<lite_motion_msgs::srv::PlanArmMotion>(plan_client_,"plan_arm_motion") &&
          (!e3_collision_object_enabled_ ||
           (waitForService<lite_motion_msgs::srv::AddSceneObject>(add_object_client_,"add_scene_object") &&
            waitForService<lite_motion_msgs::srv::RemoveSceneObject>(remove_object_client_,"remove_scene_object") &&
            waitForService<lite_motion_msgs::srv::SetSceneCollisionAllowance>(set_scene_collision_allowance_client_,"set_scene_collision_allowance")));
        if(services) {
          if(operation=="prepare") {
            success=readInitialArmJointPositions(arm_state_) &&
              executeArmStagedYawPlan(pre_initial_arm_positions_,"mission_prepare_exploration_pose");
          } else if(operation=="observe") {
            // Mission observe: empty-gripper move to tzb.observe_joint_positions (planned, collision-checked).
            // safe_to_drive stays false here; the mission sends prepare before any chassis motion.
            success=readInitialArmJointPositions(arm_state_) &&
              executeArmJointPlan(observe_joint_positions_,"mission_observe_pose");
          } else if(operation=="calib_pick") success=runCalibPick();
          else if(operation=="calib_place") success=runCalibPlaceOrRegrasp(false);
          else if(operation=="calib_regrasp") success=runCalibPlaceOrRegrasp(true);
          else if(operation=="calib_observe") success=runCalibObserve();
          else if(operation=="calib_joints") success=runCalibJoints();
          else if(operation=="calib_check") success=runCalibCheck();
          else success=runSequenceOnce();
        }
      } catch(const std::exception &e) {
        RCLCPP_ERROR(get_logger(),"Mission arm operation failed: %s",e.what());
      }
      bool canceled=false;
      {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_executing_=false;
        canceled=mission_cancel_requested_;
      }
      // The cancel abort covers only the main operation; recovery must run.
      mission_abort_.store(false);
      bool recovered=false;
      if(!success && isCalibOp(operation) && gripper_closed_on_object_.load() && !stopping()) {
        // Calibration block in the jaws: never place it on the vehicle; hold and report.
        RCLCPP_ERROR(get_logger(),"[CALIB][%s] failed with the block held: holding the measured pose (no recovery, no release)",operation.c_str());
        try { publishMeasuredHold(operation + "_failed_object_hold"); } catch(const std::exception &e) {
          RCLCPP_ERROR(get_logger(),"hold failed: %s",e.what()); }
      } else if(!success && (operation=="grasp" || isCalibOp(operation) || canceled) && !stopping()) {
        setPhase("home");
        try {
          recovered=recoverToPreIni(canceled ? "mission_cancel_recover_exploration_pose"
                                             : "mission_recover_exploration_pose_after_failed_grasp");
        } catch(const std::exception &e) {
          RCLCPP_ERROR(get_logger(),"Arm recovery failed: %s",e.what());
        }
      }
      record_tcp_trajectory_.store(false);
      finishMissionTask(success, recovered);
    }
  }

  // Geometry uses physical joint feedback at the cloud timestamp, never commanded joints
  // or the frozen camera frame. The chain itself (T_ref, axis conversion, empirical
  // base-frame correction) is the grasp chain (correctedTargetInPlanningFrame), so
  // /mission/target_center is comparable with grasp-time centers (identity gate) and
  // with the physically tuned grasp target.
  void navigationTarget(const geometry_msgs::msg::PoseStamped &in, bool precise) {
    const double t=rclcpp::Time(in.header.stamp).seconds();
    const double age=now().seconds()-t;
    if(in.header.frame_id!=navigation_sensor_frame_ || !std::isfinite(age) || age < -0.15 || age>0.6) return;
    std::vector<double> joints;
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      double error=0.10;
      for(const auto &sample: mission_joint_history_) if(std::abs(sample.first-t)<error) {
        error=std::abs(sample.first-t); joints=sample.second;
      }
      if(joints.empty() || std::chrono::duration<double>(std::chrono::steady_clock::now()-latest_arm_state_time_).count()>arm_state_max_age_sec_) return;
    }
    try {
      auto q=robot_model_->groupToFull(arm_group_,Eigen::Map<Eigen::VectorXd>(joints.data(),joints.size()),robot_model_->neutralConfiguration());
      pinocchio::Data measured(robot_model_->model());
      pinocchio::forwardKinematics(robot_model_->model(),measured,q);
      pinocchio::updateFramePlacements(robot_model_->model(),measured);
      const auto camera=measured.oMf[robot_model_->getFrameIdChecked(pose_estimator_reference_frame_)];
      geometry_msgs::msg::PoseStamped out=correctedTargetInPlanningFrame(camera,in);
      if(!std::isfinite(out.pose.position.x)||!std::isfinite(out.pose.position.y)||!std::isfinite(out.pose.position.z)) return;
      if(precise) {
        mission_centers_.push_back({t,Eigen::Vector3d(out.pose.position.x,out.pose.position.y,out.pose.position.z)});
        while(mission_centers_.size()>30) mission_centers_.pop_front();
        navigation_center_pub_->publish(out);
        // DDS does not order separate topics. A raw grasp pose may arrive before
        // its matching center; defer that frame until its identity can be checked.
        for(auto it=mission_pending_poses_.begin();it!=mission_pending_poses_.end();++it)
          if(std::abs(rclcpp::Time(it->header.stamp).seconds()-t)<0.001) {
            const auto pending=*it;
            mission_pending_poses_.erase(it);
            onPoseEstimatorTarget(pending);
            break;
          }
      } else navigation_hint_pub_->publish(out);
    } catch(const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Navigation target transform rejected: %s",e.what());
    }
  }

  std::array<double, 3> declareVec3Param(
    const std::string& name,
    const std::array<double, 3>& default_value) {
    auto values = this->declare_parameter<std::vector<double>>(
      name, std::vector<double>{default_value[0], default_value[1], default_value[2]});
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly 3 values");
    }
    return {values[0], values[1], values[2]};
  }

  Eigen::Matrix3d declareMat3Param(
    const std::string& name,
    const std::array<double, 9>& default_value) {
    auto values = this->declare_parameter<std::vector<double>>(
      name,
      std::vector<double>(default_value.begin(), default_value.end()));
    if (values.size() != 9) {
      throw std::runtime_error(name + " must contain exactly 9 values");
    }

    Eigen::Matrix3d out;
    out << values[0], values[1], values[2],
           values[3], values[4], values[5],
           values[6], values[7], values[8];
    const double orthogonality_error =
      (out.transpose() * out - Eigen::Matrix3d::Identity()).norm();
    const double det = out.determinant();
    if (orthogonality_error > 1e-3 || std::abs(det - 1.0) > 1e-3) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s is not a proper rotation matrix: det=%.6f orthogonality_error=%.6f",
        name.c_str(),
        det,
        orthogonality_error);
    }
    return out;
  }

  Eigen::Matrix4d declareMat4Param(
    const std::string& name,
    const std::array<double, 16>& default_value) {
    auto values = this->declare_parameter<std::vector<double>>(
      name,
      std::vector<double>(default_value.begin(), default_value.end()));
    if (values.size() != 16) {
      throw std::runtime_error(name + " must contain exactly 16 values");
    }

    Eigen::Matrix4d out;
    out << values[0], values[1], values[2], values[3],
           values[4], values[5], values[6], values[7],
           values[8], values[9], values[10], values[11],
           values[12], values[13], values[14], values[15];
    const Eigen::Matrix3d rot = out.block<3, 3>(0, 0);
    const double orthogonality_error =
      (rot.transpose() * rot - Eigen::Matrix3d::Identity()).norm();
    const double det = rot.determinant();
    const double bottom_row_error =
      (out.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm();
    if (orthogonality_error > 1e-3 || std::abs(det - 1.0) > 1e-3 ||
        bottom_row_error > 1e-6) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s is not a proper SE3 transform: det=%.6f orthogonality_error=%.6f bottom_row_error=%.6f",
        name.c_str(),
        det,
        orthogonality_error,
        bottom_row_error);
    }
    return out;
  }

  struct ReplaySegment {
    std::string stage_name;
    trajectory_msgs::msg::JointTrajectory trajectory;
    bool e3_attached_during_segment{false};
  };

  geometry_msgs::msg::PoseStamped makePoseStamped(
    const std::array<double, 3>& xyz,
    const std::array<double, 3>& rpy,
    const std::string& frame_id) const {
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame_id;
    out.pose.position.x = xyz[0];
    out.pose.position.y = xyz[1];
    out.pose.position.z = xyz[2];
    out.pose.orientation = quaternionMsgFromRpy(rpy[0], rpy[1], rpy[2]);
    return out;
  }

  Eigen::VectorXd currentFullState() const {
    Eigen::VectorXd q_full = robot_model_->neutralConfiguration();
    const Eigen::VectorXd q_arm = Eigen::Map<const Eigen::VectorXd>(
      arm_state_.data(), static_cast<Eigen::Index>(arm_state_.size()));
    q_full = robot_model_->groupToFull(arm_group_, q_arm, q_full);

    for (size_t i = 0; i < gripper_joint_names_.size() && i < gripper_state_.size(); ++i) {
      try {
        q_full[robot_model_->qIndexOfJoint(gripper_joint_names_[i])] = gripper_state_[i];
      } catch (const std::exception&) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Gripper joint [%s] is not an active Pinocchio joint; skipping full-state sync.",
          gripper_joint_names_[i].c_str());
      }
    }
    return q_full;
  }

  pinocchio::SE3 currentFramePose(const std::string& frame_name) {
    const Eigen::VectorXd q_full = currentFullState();
    pinocchio::forwardKinematics(robot_model_->model(), robot_model_->data(), q_full);
    pinocchio::updateFramePlacements(robot_model_->model(), robot_model_->data());
    return robot_model_->data().oMf[robot_model_->getFrameIdChecked(frame_name)];
  }

  geometry_msgs::msg::PoseStamped currentFramePoseStamped(const std::string& frame_name) {
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = planning_frame_;
    out.pose = poseMsgFromSE3(currentFramePose(frame_name));
    return out;
  }

  geometry_msgs::msg::PoseStamped transformPoseToPlanningFrame(
    const geometry_msgs::msg::PoseStamped& in) {
    if (in.header.frame_id.empty() || in.header.frame_id == planning_frame_) {
      geometry_msgs::msg::PoseStamped out = in;
      out.header.frame_id = planning_frame_;
      return out;
    }

    const pinocchio::SE3 ref_in_planning = referenceFramePoseInPlanningFrame(in.header.frame_id);
    const pinocchio::SE3 pose_in_ref = poseMsgToSE3(in.pose);
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = planning_frame_;
    out.pose = poseMsgFromSE3(ref_in_planning * pose_in_ref);
    return out;
  }

  pinocchio::SE3 referenceFramePoseInPlanningFrame(const std::string& frame_name) {
    if (apply_reference_transform_to_frozen_frame_ &&
        !frozen_camera_frame_id_.empty() &&
        frame_name == frozen_camera_frame_id_) {
      pinocchio::SE3 shadow_pose;
      if (shadowCameraFramePose(shadow_pose)) {
        return shadow_pose;
      }
    }
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (have_frozen_pose_estimator_reference_ &&
          !pose_estimator_reference_frame_.empty() &&
          frame_name == pose_estimator_reference_frame_) {
        return frozen_pose_estimator_reference_in_planning_;
      }
    }
    return currentFramePose(frame_name);
  }

  pinocchio::SE3 poseEstimatorReferenceTransformSE3() const {
    return pinocchio::SE3(
      pose_estimator_reference_transform_.block<3, 3>(0, 0),
      pose_estimator_reference_transform_.block<3, 1>(0, 3));
  }

  geometry_msgs::msg::TransformStamped transformMsgFromSE3(
    const pinocchio::SE3& tf,
    const std::string& parent_frame,
    const std::string& child_frame) const {
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = this->now();
    out.header.frame_id = parent_frame;
    out.child_frame_id = child_frame;
    out.transform.translation.x = tf.translation().x();
    out.transform.translation.y = tf.translation().y();
    out.transform.translation.z = tf.translation().z();
    const Eigen::Quaterniond q(tf.rotation());
    out.transform.rotation.x = q.x();
    out.transform.rotation.y = q.y();
    out.transform.rotation.z = q.z();
    out.transform.rotation.w = q.w();
    return out;
  }

  bool shadowCameraFramePose(pinocchio::SE3& out) {
    if (pose_estimator_reference_frame_.empty()) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (have_frozen_pose_estimator_reference_) {
        out = frozen_pose_estimator_reference_in_planning_;
        if (apply_reference_transform_to_frozen_frame_) {
          out = out * poseEstimatorReferenceTransformSE3();
        }
        return true;
      }
    }

    try {
      out = currentFramePose(pose_estimator_reference_frame_);
    } catch (const std::exception& e) {
      if (!shadow_camera_pose_error_logged_) {
        shadow_camera_pose_error_logged_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Shadow camera frame [%s] is not available before observe: %s",
          pose_estimator_reference_frame_.c_str(),
          e.what());
      }
      return false;
    }

    if (apply_reference_transform_to_frozen_frame_) {
      out = out * poseEstimatorReferenceTransformSE3();
    }
    return true;
  }

  bool shadowCameraFramePoseStamped(geometry_msgs::msg::PoseStamped& out) {
    pinocchio::SE3 pose;
    if (!shadowCameraFramePose(pose)) {
      return false;
    }
    out.header.frame_id = planning_frame_;
    out.pose = poseMsgFromSE3(pose);
    return true;
  }

  void publishFrozenCameraFrame() {
    if (!frozen_camera_tf_broadcaster_) {
      return;
    }
    pinocchio::SE3 shadow_pose;
    if (!shadowCameraFramePose(shadow_pose)) {
      return;
    }
    frozen_camera_tf_broadcaster_->sendTransform(
      transformMsgFromSE3(shadow_pose, planning_frame_, frozen_camera_frame_id_));
  }

  void transformPointCloudToReferenceFrame(sensor_msgs::msg::PointCloud2& cloud) {
    if (!transform_frozen_pointcloud_to_reference_frame_ ||
        apply_reference_transform_to_frozen_frame_) {
      return;
    }

    try {
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }
        const Eigen::Vector4d p_src(*iter_x, *iter_y, *iter_z, 1.0);
        const Eigen::Vector4d p_ref = pose_estimator_reference_transform_ * p_src;
        *iter_x = static_cast<float>(p_ref.x());
        *iter_y = static_cast<float>(p_ref.y());
        *iter_z = static_cast<float>(p_ref.z());
      }
    } catch (const std::exception& e) {
      if (!pointcloud_transform_error_logged_) {
        pointcloud_transform_error_logged_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Frozen point cloud transform skipped: %s",
          e.what());
      }
    }
  }

  void onCameraPointCloud(const sensor_msgs::msg::PointCloud2& msg) {
    if (!frozen_camera_pointcloud_pub_) {
      return;
    }
    pinocchio::SE3 shadow_pose;
    if (!shadowCameraFramePose(shadow_pose)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out = msg;
    transformPointCloudToReferenceFrame(out);
    out.header.frame_id = frozen_camera_frame_id_;
    out.header.stamp = this->now();
    frozen_camera_pointcloud_pub_->publish(out);
    publishFrozenCameraFrame();

    const auto now_mono = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_mono - last_frozen_pointcloud_log_time_).count() >
        frozen_camera_pointcloud_log_throttle_sec_) {
      last_frozen_pointcloud_log_time_ = now_mono;
      bool frozen = false;
      {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        frozen = have_frozen_pose_estimator_reference_;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Frozen camera cloud relay: source=%s source_frame=%s -> topic=%s frame=%s points=%u mode=%s",
        camera_pointcloud_topic_.c_str(),
        msg.header.frame_id.c_str(),
        frozen_camera_pointcloud_topic_.c_str(),
        frozen_camera_frame_id_.c_str(),
        msg.width * msg.height,
        frozen ? "frozen_at_observe" : "following_camera_link");
    }
  }

  geometry_msgs::msg::PoseStamped e3TargetPoseInPlanningFrame() {
    return transformPoseToPlanningFrame(
      makePoseStamped(e3_target_xyz_, e3_target_rpy_, e3_target_frame_));
  }

  void onArmState(const sensor_msgs::msg::JointState& msg) {
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    std::vector<double> positions;
    positions.reserve(arm_group.joint_names.size());
    for (const auto& joint_name : arm_group.joint_names) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
      if (it == msg.name.end()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "[GRASP][feedback][REJECT] missing_joint=%s topic=%s",
          joint_name.c_str(), arm_state_topic_.c_str());
        return;
      }
      const auto index = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (index >= msg.position.size() || !std::isfinite(msg.position[index])) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "[GRASP][feedback][REJECT] invalid_position joint=%s topic=%s",
          joint_name.c_str(), arm_state_topic_.c_str());
        return;
      }
      positions.push_back(msg.position[index]);
    }

    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      latest_measured_arm_positions_ = positions;
      if(mission_controlled_) {
        const double t=rclcpp::Time(msg.header.stamp).seconds();
        if(t>0 && (mission_joint_history_.empty() || t>mission_joint_history_.back().first)) {
          mission_joint_history_.push_back({t,positions});
          while(!mission_joint_history_.empty() && t-mission_joint_history_.front().first>3.0) mission_joint_history_.pop_front();
        }
      }
      latest_arm_state_time_ = std::chrono::steady_clock::now();
      have_complete_arm_state_ = true;
      ++arm_state_sequence_;
    }
    arm_state_cv_.notify_all();

    if (publish_tcp_trajectory_ && record_tcp_trajectory_.load()) {
      try {
        appendTcpTrajectoryPoint(measured_tcp_trajectory_, tcpPointForArmJoints(positions));
      } catch (const std::exception& e) {
        RCLCPP_WARN_ONCE(
          this->get_logger(),
          "Unable to record measured TCP trajectory: %s",
          e.what());
      }
    }
  }

  void onPoseEstimatorTarget(const geometry_msgs::msg::PoseStamped& msg) {
    if(mission_controlled_) {
      std::vector<double> expected;
      {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        if(mission_phase_!="BUSY" || !missionOperationUsesVision(mission_operation_)) return;
        expected=mission_expected_center_;
      }
      // 11-14: association gate: only reject a pose whose stamp is KNOWN in mission_centers_
      // but whose xyz disagrees with expected_center (bad navigation-chain match).
      // A pose whose stamp is not in mission_centers_ (direct grasp / T5 / no nav chain)
      // is not rejected here - it falls through to stream detection.
      {
        bool same=false, known=false;
        const double t=rclcpp::Time(msg.header.stamp).seconds();
        if(expected.size()==3) for(const auto &center:mission_centers_) {
          if(std::abs(center.first-t)<0.001) {
            known=true;
            if((center.second-Eigen::Vector3d(expected[0],expected[1],expected[2])).norm()<0.10) same=true;
          }
        }
        if(known && !same) {
          // stamp matched the nav-chain but xyz disagreed: reset stream and skip
          std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
          resetPoseEstimatorContinuityLocked();
          return;
        }
        // unknown stamp (no nav chain) OR known+same: fall through to stream detection
        if(!known && now().seconds()-t>=-0.15 && now().seconds()-t<=0.6) {
          mission_pending_poses_.push_back(msg);
          while(mission_pending_poses_.size()>30) mission_pending_poses_.pop_front();
        }
      }
    }
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (!pose_estimator_capture_enabled_ || have_pose_estimator_target_) {
        return;
      }
    }

    if (!isValidPoseEstimatorTarget(msg)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "[GRASP][target][REJECT] nonfinite_position_or_invalid_quaternion frame=%s",
        msg.header.frame_id.c_str());
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      resetPoseEstimatorContinuityLocked();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (!pose_estimator_capture_enabled_ || have_pose_estimator_target_) {
        return;
      }

      if (!pose_estimator_stream_active_ ||
          std::chrono::duration<double>(now - pose_estimator_last_msg_time_).count() >
            pose_estimator_continuous_max_gap_sec_) {
        pose_estimator_stream_active_ = true;
        pose_estimator_stream_start_time_ = now;
        pose_lock_window_.clear();
        pose_lock_retry_count_ = 0;
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "[GRASP][target][WAIT_CONTINUITY] stream started/restarted; required=%.2fs (repeated starts indicate gaps).",
          pose_estimator_continuous_required_sec_);
      }
      pose_estimator_last_msg_time_ = now;
      pose_lock_window_.push_back(msg);
      if (pose_lock_window_.size() > 240) {
        pose_lock_window_.erase(pose_lock_window_.begin());
      }

      const double continuous_sec =
        std::chrono::duration<double>(now - pose_estimator_stream_start_time_).count();
      if (continuous_sec < pose_estimator_continuous_required_sec_) {
        return;
      }

      geometry_msgs::msg::PoseStamped locked = msg;
      if (!pose_lock_window_.empty()) {
        std::vector<double> xs, ys, zs;
        xs.reserve(pose_lock_window_.size());
        ys.reserve(pose_lock_window_.size());
        zs.reserve(pose_lock_window_.size());
        for (const auto & sample : pose_lock_window_) {
          xs.push_back(sample.pose.position.x);
          ys.push_back(sample.pose.position.y);
          zs.push_back(sample.pose.position.z);
        }
        auto median_of = [](std::vector<double> & values) {
            std::sort(values.begin(), values.end());
            return values[values.size() / 2];
          };
        locked.pose.position.x = median_of(xs);
        locked.pose.position.y = median_of(ys);
        locked.pose.position.z = median_of(zs);
        const double mx = locked.pose.position.x;
        const double my = locked.pose.position.y;
        const double mz = locked.pose.position.z;
        double best_dist = std::numeric_limits<double>::infinity();
        for (const auto & sample : pose_lock_window_) {
          const double dx = sample.pose.position.x - mx;
          const double dy = sample.pose.position.y - my;
          const double dz = sample.pose.position.z - mz;
          const double dist = dx * dx + dy * dy + dz * dz;
          if (dist < best_dist) {
            best_dist = dist;
            locked.pose.orientation = sample.pose.orientation;
          }
        }
        double max_dev = 0.0;
        for (const auto & sample : pose_lock_window_) {
          const double dx = sample.pose.position.x - mx;
          const double dy = sample.pose.position.y - my;
          const double dz = sample.pose.position.z - mz;
          max_dev = std::max(max_dev, std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        RCLCPP_INFO(
          this->get_logger(),
          "[GRASP][target][AVERAGED] samples=%zu median_xyz=(%.4f, %.4f, %.4f) max_dev=%.4fm",
          pose_lock_window_.size(),
          locked.pose.position.x, locked.pose.position.y, locked.pose.position.z,
          max_dev);
        if (target_lock_max_spread_m_ > 0.0 &&
            max_dev > target_lock_max_spread_m_ && pose_lock_retry_count_ < 5) {
          ++pose_lock_retry_count_;
          RCLCPP_WARN(
            this->get_logger(),
            "[GRASP][target][UNSTABLE] max_dev=%.4fm > %.4fm; re-acquiring (%d/5).",
            max_dev, target_lock_max_spread_m_, pose_lock_retry_count_);
          pose_lock_window_.clear();
          pose_estimator_stream_start_time_ = now;
          return;
        }
      }
      latest_pose_estimator_target_ = locked;
      have_pose_estimator_target_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "[GRASP][target][LOCKED] continuous=%.3fs; later poses ignored until next acquisition.",
        continuous_sec);
      updateLockMeasuredReferenceLocked(locked);
    }
    pose_estimator_cv_.notify_all();
  }

  // Median of the measured joints over the lock window (joint samples whose stamps fall
  // inside [first, last] lock-window stamp; fallback: the sample nearest the locked
  // stamp; fallback: the latest measured joints). Caller holds pose_estimator_mutex_;
  // arm_state_mutex_ is a leaf lock (never held while pose_estimator_mutex_ is taken).
  void updateLockMeasuredReferenceLocked(const geometry_msgs::msg::PoseStamped& locked) {
    lock_measured_joints_valid_ = false;
    std::vector<std::vector<double>> samples;
    std::vector<double> nearest, latest;
    {
      double t_first = rclcpp::Time(locked.header.stamp).seconds();
      double t_last = t_first;
      for (const auto& s : pose_lock_window_) {
        const double t = rclcpp::Time(s.header.stamp).seconds();
        t_first = std::min(t_first, t);
        t_last = std::max(t_last, t);
      }
      const double t_lock = rclcpp::Time(locked.header.stamp).seconds();
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      if (!have_complete_arm_state_) return;
      latest = latest_measured_arm_positions_;
      double best = 1e9;
      for (const auto& h : mission_joint_history_) {
        if (h.first >= t_first - 0.02 && h.first <= t_last + 0.02) samples.push_back(h.second);
        if (std::abs(h.first - t_lock) < best) { best = std::abs(h.first - t_lock); nearest = h.second; }
      }
      if (best > 0.5) nearest.clear();
    }
    std::vector<double> joints;
    std::string source;
    if (samples.size() >= 3) {
      joints.assign(samples.front().size(), 0.0);
      for (size_t k = 0; k < joints.size(); ++k) {
        std::vector<double> col;
        col.reserve(samples.size());
        for (const auto& s : samples) if (k < s.size()) col.push_back(s[k]);
        std::nth_element(col.begin(), col.begin() + col.size() / 2, col.end());
        joints[k] = col[col.size() / 2];
      }
      source = "median";
    } else if (!nearest.empty()) {
      joints = nearest;
      source = "nearest";
    } else {
      joints = latest;
      source = "latest";
    }
    try {
      lock_measured_camera_in_planning_ = framePoseAtArmJoints(pose_estimator_reference_frame_, joints);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[GRASP][target][LOCKED] measured-joint camera FK failed: %s", e.what());
      return;
    }
    lock_measured_joints_ = joints;
    lock_measured_joints_valid_ = true;
    double delta_mm = -1.0;
    if (have_frozen_pose_estimator_reference_) {
      delta_mm = 1000.0 * (lock_measured_camera_in_planning_.translation() -
                           frozen_pose_estimator_reference_in_planning_.translation()).norm();
    }
    if (camera_frame_from_measured_joints_) {
      frozen_pose_estimator_reference_in_planning_ = lock_measured_camera_in_planning_;
      have_frozen_pose_estimator_reference_ = true;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][target][LOCKED] measured joints (%s of %zu samples) J1..J6=(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f); "
      "camera_link FK delta vs frozen reference=%.1f mm; %s",
      source.c_str(), samples.size(),
      joints.size() > 0 ? joints[0] : 0.0, joints.size() > 1 ? joints[1] : 0.0, joints.size() > 2 ? joints[2] : 0.0,
      joints.size() > 3 ? joints[3] : 0.0, joints.size() > 4 ? joints[4] : 0.0, joints.size() > 5 ? joints[5] : 0.0,
      delta_mm,
      camera_frame_from_measured_joints_ ? "APPLIED as the frozen reference (tzb.camera_frame_from_measured_joints=true)"
                                         : "report only (tzb.camera_frame_from_measured_joints=false)");
  }

  void resetPoseEstimatorCaptureLocked() {
    have_pose_estimator_target_ = false;
    pose_estimator_stream_active_ = false;
    pose_estimator_capture_enabled_ = false;
    latest_pose_estimator_target_ = geometry_msgs::msg::PoseStamped();
    pose_lock_window_.clear();
    pose_lock_retry_count_ = 0;
  }

  bool enablePoseEstimatorCaptureAtCurrentReferencePose() {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    resetPoseEstimatorCaptureLocked();
    have_frozen_pose_estimator_reference_ = false;
    pose_estimator_capture_enabled_ = true;
    RCLCPP_DEBUG(
      this->get_logger(),
      "Pose estimator capture enabled; reference frame will be frozen after reading the physical arm state: %s",
      pose_estimator_reference_frame_.empty() ? "<message_header>" : pose_estimator_reference_frame_.c_str());
    return true;
  }

  pinocchio::SE3 framePoseAtArmJoints(
    const std::string& frame_name, const std::vector<double>& arm_joints) {
    Eigen::VectorXd q_full = robot_model_->neutralConfiguration();
    const Eigen::VectorXd q_arm = Eigen::Map<const Eigen::VectorXd>(
      arm_joints.data(), static_cast<Eigen::Index>(arm_joints.size()));
    q_full = robot_model_->groupToFull(arm_group_, q_arm, q_full);
    pinocchio::Data local_data(robot_model_->model());
    pinocchio::forwardKinematics(robot_model_->model(), local_data, q_full);
    pinocchio::updateFramePlacements(robot_model_->model(), local_data);
    return local_data.oMf[robot_model_->getFrameIdChecked(frame_name)];
  }

  bool freezePoseEstimatorReferenceAtCurrentArmState() {
    if (pose_estimator_reference_frame_.empty()) {
      return true;
    }
    try {
      const bool use_planned =
        freeze_reference_from_planned_observe_ && !observed_reference_joints_.empty() &&
        !camera_frame_from_measured_joints_;
      const auto reference_pose = use_planned
        ? framePoseAtArmJoints(pose_estimator_reference_frame_, observed_reference_joints_)
        : currentFramePose(pose_estimator_reference_frame_);
      {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        frozen_pose_estimator_reference_in_planning_ = reference_pose;
        have_frozen_pose_estimator_reference_ = true;
      }
      if (use_planned) {
        double delta_mm = 0.0;
        double delta_tilt_deg = 0.0;
        try {
          const auto measured_pose = currentFramePose(pose_estimator_reference_frame_);
          delta_mm = 1000.0 * (reference_pose.translation() - measured_pose.translation()).norm();
          const Eigen::Matrix3d dR = measured_pose.rotation().transpose() * reference_pose.rotation();
          delta_tilt_deg = std::abs(Eigen::AngleAxisd(dR).angle()) * 180.0 / M_PI;
        } catch (const std::exception&) {
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Pose estimator reference frame [%s] frozen from the PLANNED observe joints "
          "(measured-vs-planned delta=%.1f mm %.2f deg).",
          pose_estimator_reference_frame_.c_str(), delta_mm, delta_tilt_deg);
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Pose estimator reference frame [%s] frozen from the measured arm state.",
          pose_estimator_reference_frame_.c_str());
      }
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to freeze pose estimator reference frame [%s] in [%s]: %s",
        pose_estimator_reference_frame_.c_str(),
        planning_frame_.c_str(),
        e.what());
      return false;
    }
  }

  bool isValidPoseEstimatorTarget(const geometry_msgs::msg::PoseStamped& msg) const {
    const auto& p = msg.pose.position;
    const auto& q = msg.pose.orientation;
    const double q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) &&
           q_norm_sq > 1e-12;
  }

  void resetPoseEstimatorContinuityLocked() {
    if (have_pose_estimator_target_) {
      return;
    }
    if (pose_estimator_stream_active_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Pose estimator target stream continuity reset because an invalid pose was received.");
    }
    pose_estimator_stream_active_ = false;
  }

  geometry_msgs::msg::PoseStamped applyPoseEstimatorAxisConversion(
    const geometry_msgs::msg::PoseStamped& in) const {
    geometry_msgs::msg::PoseStamped out = in;
    const Eigen::Matrix3d converted_rotation =
      quaternionEigenFromMsg(in.pose.orientation).toRotationMatrix() *
      pose_estimator_axis_conversion_;
    Eigen::Quaterniond q(converted_rotation);
    q.normalize();
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
    return out;
  }

  geometry_msgs::msg::PoseStamped applyPoseEstimatorReferenceTransform(
    const geometry_msgs::msg::PoseStamped& in) const {
    if (pose_estimator_reference_frame_.empty()) {
      return in;
    }
    if (apply_reference_transform_to_frozen_frame_) {
      geometry_msgs::msg::PoseStamped out = in;
      out.header.frame_id = frozen_camera_frame_id_;
      return out;
    }

    const pinocchio::SE3 reference_from_estimator = poseEstimatorReferenceTransformSE3();
    geometry_msgs::msg::PoseStamped out;
    out.header = in.header;
    out.header.frame_id = pose_estimator_reference_frame_;
    out.pose = poseMsgFromSE3(reference_from_estimator * poseMsgToSE3(in.pose));
    return out;
  }

  geometry_msgs::msg::PoseStamped preparePoseEstimatorTarget(
    const geometry_msgs::msg::PoseStamped& in) const {
    return applyPoseEstimatorAxisConversion(applyPoseEstimatorReferenceTransform(in));
  }

  bool waitForPoseEstimatorTarget(geometry_msgs::msg::PoseStamped& out) {
    std::unique_lock<std::mutex> lock(pose_estimator_mutex_);
    const auto has_target = [this]() {
      return stopping() || have_pose_estimator_target_;
    };
    if (!has_target()) {
      RCLCPP_INFO(
        this->get_logger(),
        "[GRASP][target][WAIT] topic=%s timeout=%.2fs continuous_required=%.2fs (timeout<=0 means indefinite).",
        pose_estimator_target_topic_.c_str(),
        pose_estimator_wait_timeout_sec_,
        pose_estimator_continuous_required_sec_);
      if (pose_estimator_wait_timeout_sec_ <= 0.0) {
        pose_estimator_cv_.wait(lock, has_target);
      } else if (!pose_estimator_cv_.wait_for(
          lock,
          std::chrono::duration<double>(pose_estimator_wait_timeout_sec_),
          has_target)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[GRASP][target][TIMEOUT] no locked target topic=%s timeout=%.2fs.",
          pose_estimator_target_topic_.c_str(),
          pose_estimator_wait_timeout_sec_);
        return false;
      }
    }
    if (stopping() || !have_pose_estimator_target_) {
      return false;
    }
    out = latest_pose_estimator_target_;
    return true;
  }

  void applyTargetCorrection(geometry_msgs::msg::PoseStamped& pose) {
    std::vector<double> corr_rpy{0.0, 0.0, 0.0};
    std::vector<double> corr_xyz{0.0, 0.0, 0.0};
    double corr_min_z = -1.0;
    this->get_parameter("tzb.target_correction_rpy_deg", corr_rpy);
    this->get_parameter("tzb.target_correction_xyz", corr_xyz);
    this->get_parameter("tzb.target_min_z", corr_min_z);
    if (corr_rpy.size() != 3 || corr_xyz.size() != 3) {
      return;
    }
    target_correction_rpy_deg_ << corr_rpy[0], corr_rpy[1], corr_rpy[2];
    target_correction_xyz_ << corr_xyz[0], corr_xyz[1], corr_xyz[2];
    target_min_z_ = corr_min_z;
    const double r = target_correction_rpy_deg_(0) * M_PI / 180.0;
    const double p = target_correction_rpy_deg_(1) * M_PI / 180.0;
    const double y = target_correction_rpy_deg_(2) * M_PI / 180.0;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    Eigen::Matrix3d R;
    R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
         -sp,     cp * sr,                cp * cr;
    Eigen::Vector3d v(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    v = R * v + target_correction_xyz_;
    if (v.z() < target_min_z_) {
      v.z() = target_min_z_;
    }
    pose.pose.position.x = v.x();
    pose.pose.position.y = v.y();
    pose.pose.position.z = v.z();
    if (target_left_y_threshold_ > 0.0 &&
        pose.pose.position.y > target_left_y_threshold_) {
      pose.pose.position.y += target_left_y_offset_;
    }
    // 这个修正是 base 系的一次刚体变换 v' = R*v + t, 姿态必须跟着转,
    // 否则位置对了、夹爪却固定差 R 的那个角(实测 ~25deg)。
    // 与 target_bridge.py 的 out_R = corr_R @ robot_R 保持一致。
    Eigen::Quaterniond corrected_q = quaternionEigenFromMsg(pose.pose.orientation);
    corrected_q = Eigen::Quaterniond(R) * corrected_q;
    corrected_q.normalize();
    pose.pose.orientation.x = corrected_q.x();
    pose.pose.orientation.y = corrected_q.y();
    pose.pose.orientation.z = corrected_q.z();
    pose.pose.orientation.w = corrected_q.w();
  }

  // Pose of pose_estimator_reference_frame_ (camera_link) in the planning
  // frame as the grasp pipeline uses it: frozen at observe when a freeze
  // exists, otherwise the current (commanded-state) FK. Never includes T_ref.
  pinocchio::SE3 graspReferencePoseInPlanningFrame() {
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (have_frozen_pose_estimator_reference_) {
        return frozen_pose_estimator_reference_in_planning_;
      }
    }
    return currentFramePose(pose_estimator_reference_frame_);
  }

  // The single sensor->planning-frame target chain shared by the grasp path
  // (resolveE3TargetPose) and the navigation relay (navigationTarget):
  //   planning_T_target = reference_in_planning * T_ref * sensed_pose,
  //   orientation post-multiplied by the axis conversion, then the empirical
  //   base-frame correction (applyTargetCorrection).
  // `reference_in_planning` is the camera_link pose WITHOUT T_ref; whether
  // T_ref is multiplied here or inside preparePoseEstimatorTarget depends on
  // tzb.apply_reference_transform_to_frozen_frame, exactly as the frozen-frame
  // grasp transform did, so the two callers cannot diverge.
  geometry_msgs::msg::PoseStamped correctedTargetInPlanningFrame(
    const pinocchio::SE3& reference_in_planning,
    const geometry_msgs::msg::PoseStamped& sensed) {
    const geometry_msgs::msg::PoseStamped prepared = preparePoseEstimatorTarget(sensed);
    pinocchio::SE3 reference = reference_in_planning;
    if (apply_reference_transform_to_frozen_frame_) {
      reference = reference * poseEstimatorReferenceTransformSE3();
    }
    geometry_msgs::msg::PoseStamped out;
    out.header = sensed.header;
    out.header.frame_id = planning_frame_;
    out.pose = poseMsgFromSE3(reference * poseMsgToSE3(prepared.pose));
    applyTargetCorrection(out);
    return out;
  }

  bool lockedPoseEstimatorTargetInPlanningFrame(geometry_msgs::msg::PoseStamped& out) {
    geometry_msgs::msg::PoseStamped sensed_pose;
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (!have_pose_estimator_target_) {
        return false;
      }
      sensed_pose = latest_pose_estimator_target_;
    }

    geometry_msgs::msg::PoseStamped prepared_pose;
    try {
      prepared_pose = preparePoseEstimatorTarget(sensed_pose);
      out = transformPoseToPlanningFrame(prepared_pose);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to transform locked pose estimator target from source_frame [%s] prepared_frame [%s] to [%s] for visualization: %s",
        sensed_pose.header.frame_id.c_str(),
        prepared_pose.header.frame_id.c_str(),
        planning_frame_.c_str(),
        e.what());
      return false;
    }

    return true;
  }

  bool resolveE3TargetPose(geometry_msgs::msg::PoseStamped& out) {
    if (use_yaml_e3_target_) {
      out = e3TargetPoseInPlanningFrame();
      RCLCPP_INFO(
        this->get_logger(),
        "Using YAML E3 target: source_frame=[%s] source_xyz=(%.4f, %.4f, %.4f) source_rpy=(%.4f, %.4f, %.4f) rad -> planning_frame=[%s] xyz=(%.4f, %.4f, %.4f).",
        e3_target_frame_.c_str(),
        e3_target_xyz_[0],
        e3_target_xyz_[1],
        e3_target_xyz_[2],
        e3_target_rpy_[0],
        e3_target_rpy_[1],
        e3_target_rpy_[2],
        planning_frame_.c_str(),
        out.pose.position.x,
        out.pose.position.y,
        out.pose.position.z);
    } else {
      geometry_msgs::msg::PoseStamped sensed_pose;
      if (!waitForPoseEstimatorTarget(sensed_pose)) {
        return false;
      }
      try {
        out = correctedTargetInPlanningFrame(graspReferencePoseInPlanningFrame(), sensed_pose);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to transform pose estimator target from source_frame [%s] reference_frame [%s] to [%s]: %s",
          sensed_pose.header.frame_id.c_str(),
          pose_estimator_reference_frame_.c_str(),
          planning_frame_.c_str(),
          e.what());
        return false;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "[GRASP][transform][OK] source_frame=%s source_xyz=(%.4f, %.4f, %.4f) -> reference_frame=%s -> %s_xyz=(%.4f, %.4f, %.4f).",
        sensed_pose.header.frame_id.c_str(),
        sensed_pose.pose.position.x,
        sensed_pose.pose.position.y,
        sensed_pose.pose.position.z,
        pose_estimator_reference_frame_.c_str(),
        planning_frame_.c_str(),
        out.pose.position.x,
        out.pose.position.y,
        out.pose.position.z);
    }

    if (std::isfinite(grasp_ground_z_)) {
      const double visual_z = out.pose.position.z;
      if (std::abs(visual_z - grasp_ground_z_) > grasp_ground_z_max_deviation_) {
        RCLCPP_ERROR(this->get_logger(),
          "[GRASP][target][GROUND_Z][REJECT] visual z=%.4f is %.3f m from tzb.grasp_ground_z=%.4f (max %.3f); not grasping an inconsistent target.",
          visual_z, visual_z - grasp_ground_z_, grasp_ground_z_, grasp_ground_z_max_deviation_);
        return false;
      }
      if (!grasp_use_visual_z_) out.pose.position.z = grasp_ground_z_;
      RCLCPP_INFO(this->get_logger(),
        "[GRASP][target][GROUND_Z] visual z=%.4f -> %s z=%.4f (delta %.3f m)",
        visual_z, grasp_use_visual_z_ ? "visual" : "ground-resting",
        grasp_use_visual_z_ ? visual_z : grasp_ground_z_,
        (grasp_use_visual_z_ ? visual_z : grasp_ground_z_) - visual_z);
    }
    const bool any_forced_target_rpy =
      force_target_roll_ || force_target_pitch_ || force_target_yaw_;
    if (any_forced_target_rpy) {
      // 注意约定: quaternionMsgFromRpy 用 R = Rz(yaw)*Ry(pitch)*Rx(roll),
      // 所以取角必须用 eulerAngles(2,1,0)（返回 yaw,pitch,roll）。
      // 原来用 eulerAngles(0,1,2) 是 R = Rx*Ry*Rz 的约定, 与重组不一致,
      // 俯视抓取下会把 yaw 的符号翻掉(物体转 θ, 夹爪却转到 -θ)。
      const Eigen::Vector3d zyx_rpy =
        quaternionEigenFromMsg(out.pose.orientation).toRotationMatrix().eulerAngles(2, 1, 0);
      Eigen::Vector3d visual_target_rpy(zyx_rpy.z(), zyx_rpy.y(), zyx_rpy.x());
      if (target_yaw_symmetry_deg_ > 1e-6 && force_target_roll_ && force_target_pitch_ && !force_target_yaw_) {
        const double cube_yaw = symmetricFaceYaw(
          quaternionEigenFromMsg(out.pose.orientation).toRotationMatrix(),
          out.pose.position.x, out.pose.position.y);
        RCLCPP_INFO(this->get_logger(),
          "[GRASP][target][YAW] zyx_yaw=%.1fdeg -> face_yaw=%.1fdeg (symmetry %.0fdeg, reference radial %.1fdeg + offset %.1fdeg - grasp_yaw_offset %.1fdeg)",
          visual_target_rpy.z() * 180.0 / M_PI, cube_yaw * 180.0 / M_PI, target_yaw_symmetry_deg_,
          std::atan2(out.pose.position.y, out.pose.position.x) * 180.0 / M_PI,
          target_yaw_reference_offset_deg_, grasp_yaw_offset_deg_);
        visual_target_rpy.z() = cube_yaw;
      }
      Eigen::Vector3d target_rpy = visual_target_rpy;
      if (force_target_roll_) {
        target_rpy.x() = forced_target_roll_;
      }
      if (force_target_pitch_) {
        target_rpy.y() = forced_target_pitch_;
      }
      if (force_target_yaw_) {
        target_rpy.z() = forced_target_yaw_;
      }
      out.pose.orientation = quaternionMsgFromRpy(
        target_rpy.x(), target_rpy.y(), target_rpy.z());
      RCLCPP_DEBUG(
        this->get_logger(),
        "Resolved target RPY in [%s]: visual=(%.4f, %.4f, %.4f) rad, final=(%.4f, %.4f, %.4f) rad, forced_axes=(roll:%s pitch:%s yaw:%s).",
        planning_frame_.c_str(),
        visual_target_rpy.x(),
        visual_target_rpy.y(),
        visual_target_rpy.z(),
        target_rpy.x(),
        target_rpy.y(),
        target_rpy.z(),
        force_target_roll_ ? "roll" : "visual",
        force_target_pitch_ ? "pitch" : "visual",
        force_target_yaw_ ? "yaw" : "visual");
    }
    // 只调姿态、不动位置的 yaw 微调钮（绕 base_link z 轴，正=俯视逆时针）。
    // 用于吃掉坐标系补偿之后剩下的那点常量角度偏差。
    if (std::abs(grasp_yaw_offset_deg_) > 1e-9) {
      const Eigen::AngleAxisd yaw_offset(
        grasp_yaw_offset_deg_ * M_PI / 180.0, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q = yaw_offset * quaternionEigenFromMsg(out.pose.orientation);
      q.normalize();
      out.pose.orientation.x = q.x();
      out.pose.orientation.y = q.y();
      out.pose.orientation.z = q.z();
      out.pose.orientation.w = q.w();
      RCLCPP_DEBUG(
        this->get_logger(),
        "Applied grasp yaw offset %.2f deg in [%s].",
        grasp_yaw_offset_deg_, planning_frame_.c_str());
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][target][READY] TCP frame=%s xyz=(%.4f, %.4f, %.4f) quat_xyzw=(%.5f, %.5f, %.5f, %.5f).",
      planning_frame_.c_str(),
      out.pose.position.x,
      out.pose.position.y,
      out.pose.position.z,
      out.pose.orientation.x, out.pose.orientation.y,
      out.pose.orientation.z, out.pose.orientation.w);
    return true;
  }

  geometry_msgs::msg::Pose computeAttachRelativePose(
    const std::string& frame_name,
    const geometry_msgs::msg::PoseStamped& object_world_pose) {
    const pinocchio::SE3 frame_world = currentFramePose(frame_name);
    const pinocchio::SE3 object_world = poseMsgToSE3(object_world_pose.pose);
    return poseMsgFromSE3(frame_world.actInv(object_world));
  }

  sensor_msgs::msg::JointState makeJointStateMsg(
    const Eigen::VectorXd& q_full,
    const rclcpp::Time& stamp) const {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = robot_model_->activeJointNames();
    msg.position.reserve(msg.name.size());
    for (const auto& joint_name : msg.name) {
      msg.position.push_back(q_full[robot_model_->qIndexOfJoint(joint_name)]);
    }
    return msg;
  }

  void publishCurrentTheoreticalJointState(const std::string& stage_name) {
    if (!joint_state_pub_ || !enable_visualization_) {
      return;
    }
    joint_state_pub_->publish(makeJointStateMsg(currentFullState(), this->now()));
    RCLCPP_DEBUG(
      this->get_logger(),
      "Stage [%s] published current theoretical joint state.",
      stage_name.c_str());
  }

  bool publishGroupHoldTrajectory(
    const std::string& group_name,
    const std::vector<double>& target_joints,
    const std::string& stage_name,
    bool record_for_replay = false) {
    if (!trajectory_pub_) {
      return true;
    }

    const auto& group = robot_model_->getPlanningGroup(group_name);
    if (target_joints.size() != group.joint_names.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Stage [%s] hold trajectory size mismatch: target=%zu group=%zu",
        stage_name.c_str(),
        target_joints.size(),
        group.joint_names.size());
      return false;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = this->now();
    trajectory.joint_names = group.joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_joints;
    point.velocities.assign(target_joints.size(), 0.0);
    point.accelerations.assign(target_joints.size(), 0.0);
    point.time_from_start = durationFromSec(std::max(0.02, post_action_wait_sec_));
    trajectory.points.push_back(std::move(point));

    trajectory_pub_->publish(trajectory);
    if (record_for_replay) {
      recordReplaySegment(stage_name, trajectory);
    }
    return true;
  }

  void publishGripperCommand(
    const std::string& stage_name,
    int8_t serial_command) {
    std_msgs::msg::Int8 command_msg;
    if (serial_command != -1 && serial_command != 1 &&
        serial_command != 2 && serial_command != 3) {
      throw std::invalid_argument(
        "gripper command must be -1 (open) or 1/2/3 (closed mode 1/2/3)");
    }
    command_msg.data = serial_command;
    gripper_command_pub_->publish(command_msg);

    const bool closing = command_msg.data > 0;
    gripper_state_ = closing ? gripper_close_positions_ : gripper_open_positions_;
    // The grasp sequence closes only at the target, so a close means "object
    // in the jaws" until the next open (place release or observe open).
    gripper_closed_on_object_.store(closing);
    const std::string gripper_action = closing
      ? "closed level " + std::to_string(static_cast<int>(command_msg.data))
      : "open";
    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][gripper][SENT] step=%s serial_int8=%d (%s); physical_confirmation=unavailable.",
      stage_name.c_str(),
      static_cast<int>(command_msg.data),
      gripper_action.c_str());
    publishCurrentTheoreticalJointState(stage_name + "_final_sync");
  }

  template<typename ServiceT>
  bool waitForService(
    const typename rclcpp::Client<ServiceT>::SharedPtr& client,
    const std::string& service_name) {
    RCLCPP_INFO(this->get_logger(), "[GRASP][service][WAIT] name=%s timeout=%.2fs",
      service_name.c_str(), service_wait_sec_);
    if (client->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
      RCLCPP_INFO(this->get_logger(), "[GRASP][service][READY] name=%s", service_name.c_str());
      return true;
    }
    RCLCPP_ERROR(
      this->get_logger(),
      "[GRASP][service][UNAVAILABLE] name=%s timeout=%.2fs",
      service_name.c_str(),
      service_wait_sec_);
    return false;
  }

  template<typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> callService(
    const typename rclcpp::Client<ServiceT>::SharedPtr& client,
    const std::shared_ptr<typename ServiceT::Request>& request,
    const std::string& service_name) {
    RCLCPP_DEBUG(this->get_logger(), "[GRASP][service][CALL] name=%s timeout=%.2fs",
      service_name.c_str(), response_wait_sec_);
    auto future = client->async_send_request(request);
    // Wait in slices so an abort (shutdown) does not sit on a dead service
    // for the whole response timeout. The unattended shutdown/cancel recovery
    // gets a short budget: a planner that was SIGKILLed is still listed in the
    // ROS graph for a while, so wait_for_service() cannot catch it.
    const double response_wait_sec =
      restricted_recovery_.load() ? std::min(response_wait_sec_, kRestrictedResponseWaitSec) : response_wait_sec_;
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(response_wait_sec);
    while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
      if (stopping()) {
        RCLCPP_WARN(
          this->get_logger(),
          "[GRASP][service][ABORT] name=%s abandoned because the operation was aborted",
          service_name.c_str());
        return nullptr;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[GRASP][service][TIMEOUT] name=%s response_wait=%.2fs",
          service_name.c_str(),
          response_wait_sec);
        last_service_timeout_ = true;
        return nullptr;
      }
    }
    return future.get();
  }

  bool addMeshObject(
    const std::string& object_id,
    const std::string& mesh_resource,
    const geometry_msgs::msg::PoseStamped& object_pose) {
    RCLCPP_INFO(this->get_logger(), "[GRASP][scene][BEGIN] service=add_scene_object object=%s",
      object_id.c_str());
    auto req = std::make_shared<lite_motion_msgs::srv::AddSceneObject::Request>();
    req->object_id = object_id;
    req->mesh_resource = mesh_resource;
    req->pose = object_pose;
    req->collision_pose = object_pose.pose;
    req->mesh_scale = mesh_scale_;

    auto resp = callService<lite_motion_msgs::srv::AddSceneObject>(
      add_object_client_, req, "add_scene_object");
    if (!resp || !resp->success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Add scene object [%s] failed: %s",
        object_id.c_str(),
        resp ? resp->message.c_str() : "no response");
      return false;
    }
    RCLCPP_DEBUG(this->get_logger(), "Scene object [%s] loaded from [%s]", object_id.c_str(), mesh_resource.c_str());
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool removeObject(const std::string& object_id, bool warn_on_failure = true) {
    auto req = std::make_shared<lite_motion_msgs::srv::RemoveSceneObject::Request>();
    req->object_id = object_id;
    auto resp = callService<lite_motion_msgs::srv::RemoveSceneObject>(
      remove_object_client_, req, "remove_scene_object");
    if (!resp || !resp->success) {
      if (warn_on_failure) {
        RCLCPP_WARN(
          this->get_logger(),
          "Remove scene object [%s] failed or skipped: %s",
          object_id.c_str(),
          resp ? resp->message.c_str() : "no response");
      }
      return false;
    }
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool setObjectAttached(
    const std::string& object_id,
    bool attach,
    const std::string& frame_id,
    const geometry_msgs::msg::Pose* relative_pose = nullptr,
    bool warn_on_failure = true) {
    auto req = std::make_shared<lite_motion_msgs::srv::AttachSceneObject::Request>();
    req->object_id = object_id;
    req->attach = attach;
    req->frame_id = frame_id;
    if (relative_pose != nullptr) {
      req->relative_pose = *relative_pose;
    } else {
      req->relative_pose.orientation.w = 1.0;
    }

    auto resp = callService<lite_motion_msgs::srv::AttachSceneObject>(
      attach_object_client_, req, "attach_scene_object");
    if (!resp || !resp->success) {
      if (warn_on_failure) {
        RCLCPP_ERROR(
          this->get_logger(),
          "%s object [%s] failed: %s",
          attach ? "Attach" : "Detach",
          object_id.c_str(),
          resp ? resp->message.c_str() : "no response");
      }
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "%s [%s] %s [%s]",
      attach ? "Attached" : "Detached",
      object_id.c_str(),
      attach ? "to" : "from",
      frame_id.c_str());
    if (object_id == e3_object_id_) {
      e3_attached_to_tcp_ = attach;
      if (attach && relative_pose != nullptr) {
        e3_relative_pose_in_tcp_ = *relative_pose;
      }
      publishReferenceMarkers(attach ? "after_attach_e3" : "after_detach_e3");
      publishE3Marker();
    }
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool setRuntimeSceneCollisionAllowance(
    const std::vector<std::string>& object_ids,
    const std::vector<std::string>& link_names,
    double allowed_penetration_m,
    const std::string& stage_name) {
    auto req = std::make_shared<lite_motion_msgs::srv::SetSceneCollisionAllowance::Request>();
    req->object_ids = object_ids;
    req->link_names = link_names;
    req->allowed_penetration = allowed_penetration_m;

    auto resp = callService<lite_motion_msgs::srv::SetSceneCollisionAllowance>(
      set_scene_collision_allowance_client_, req, "set_scene_collision_allowance");
    if (!resp || !resp->success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Set scene collision allowance failed at stage [%s]: %s",
        stage_name.c_str(),
        resp ? resp->message.c_str() : "no response");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][scene][READY] step=%s objects=%zu links=%zu allowed_penetration=%.4fm",
      stage_name.c_str(),
      object_ids.size(),
      link_names.size(),
      allowed_penetration_m);
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool setGripperE3CollisionAllowance(double allowed_penetration_m, const std::string& stage_name) {
    return setRuntimeSceneCollisionAllowance(
      std::vector<std::string>{e3_object_id_},
      grasp_collision_links_,
      allowed_penetration_m,
      stage_name);
  }

  double effectiveExecutionWaitSec(const trajectory_msgs::msg::JointTrajectory& traj) const {
    double wait_sec = trajectoryDurationSec(traj);
    if (visualization_playback_enabled_) {
      wait_sec /= visualization_playback_speed_scale_;
    }
    return wait_sec + post_action_wait_sec_ + 0.25;
  }

  double maxJointError(
    const std::vector<double>& lhs,
    const std::vector<double>& rhs) const {
    if (lhs.size() != rhs.size()) {
      return std::numeric_limits<double>::infinity();
    }
    double max_err = 0.0;
    for (size_t i = 0; i < lhs.size(); ++i) {
      // Joint feedback can use an equivalent representation one or more full
      // turns away from the planned angle. Arrival validation must not reject
      // a physically identical pose solely because of a 2*pi offset.
      const double error = std::abs(std::remainder(lhs[i] - rhs[i], 2.0 * kPi));
      max_err = std::max(max_err, error);
    }
    return max_err;
  }

  void logArmJointState(const std::string& stage_name, const std::vector<double>& joints) const {
    std::ostringstream oss;
    oss << "Stage [" << stage_name << "] arm joints:";
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    for (size_t i = 0; i < joints.size() && i < group.joint_names.size(); ++i) {
      oss << " " << group.joint_names[i] << "=" << joints[i];
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  double jointToleranceRad(const std::string& joint_name) const {
    if (j6_arrival_tolerance_rad_ > 0.0 &&
        (joint_name == "J6_joint" || joint_name == "J6_Link" || joint_name == "J6_link")) {
      return j6_arrival_tolerance_rad_;
    }
    return arm_motion_goal_tolerance_rad_;
  }

  double maxNormalizedJointError(
    const std::vector<double>& measured,
    const std::vector<double>& target) const {
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    const size_t n = std::min({measured.size(), target.size(), group.joint_names.size()});
    double worst = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double err = std::abs(std::remainder(measured[i] - target[i], 2.0 * kPi));
      const double tol = std::max(1e-6, jointToleranceRad(group.joint_names[i]));
      worst = std::max(worst, err / tol);
    }
    return worst;
  }

  std::string formatArmJointErrors(
    const std::vector<double>& measured,
    const std::vector<double>& target) const {
    std::ostringstream oss;
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    const size_t n = std::min({measured.size(), target.size(), group.joint_names.size()});
    for (size_t i = 0; i < n; ++i) {
      const double raw_error = measured[i] - target[i];
      const double error = std::remainder(raw_error, 2.0 * kPi);
      const double abs_error = std::abs(error);
      oss << " " << group.joint_names[i]
          << "{target=" << target[i]
          << ", measured=" << measured[i]
          << ", error=" << error
          << ", raw_error=" << raw_error
          << ", abs=" << abs_error
          << (abs_error > jointToleranceRad(group.joint_names[i]) ? ", BAD}" : ", ok}");
    }
    if (measured.size() != target.size()) {
      oss << " size_mismatch{target=" << target.size()
          << ", measured=" << measured.size() << "}";
    }
    return oss.str();
  }

  std::vector<double> clampArmJointsToLimits(
    const std::vector<double>& joints,
    const std::string& stage_name,
    const std::string& role) const {
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    if (joints.size() != group.joint_names.size()) {
      return joints;
    }

    const auto lower = robot_model_->lowerBoundsForGroup(arm_group_);
    const auto upper = robot_model_->upperBoundsForGroup(arm_group_);
    std::vector<double> clamped = joints;
    std::ostringstream oss;
    bool adjusted = false;
    for (size_t i = 0; i < clamped.size(); ++i) {
      const double before = clamped[i];
      const double bounded = std::clamp(
        before,
        lower[static_cast<Eigen::Index>(i)],
        upper[static_cast<Eigen::Index>(i)]);
      if (std::abs(bounded - before) > 1e-9) {
        if (!adjusted) {
          oss << "Stage [" << stage_name << "] clamped " << role
              << " joints to URDF limits:";
          adjusted = true;
        }
        oss << " " << group.joint_names[i] << "=" << before << "->" << bounded;
        clamped[i] = bounded;
      }
    }
    if (adjusted) {
      RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
    }
    return clamped;
  }

  geometry_msgs::msg::Point tcpPointForArmJoints(
    const std::vector<double>& joints) const {
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    if (joints.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("TCP FK joint vector size does not match arm group DOF");
    }

    const Eigen::VectorXd q_group = Eigen::Map<const Eigen::VectorXd>(
      joints.data(), static_cast<Eigen::Index>(joints.size()));
    const Eigen::VectorXd q_full = robot_model_->groupToFull(
      arm_group_, q_group, robot_model_->neutralConfiguration());
    pinocchio::Data data(robot_model_->model());
    pinocchio::forwardKinematics(robot_model_->model(), data, q_full);
    pinocchio::updateFramePlacements(robot_model_->model(), data);
    const auto& tcp_pose = data.oMf[robot_model_->getFrameIdChecked(tcp_frame_)];

    geometry_msgs::msg::Point point;
    point.x = tcp_pose.translation().x();
    point.y = tcp_pose.translation().y();
    point.z = tcp_pose.translation().z();
    return point;
  }

  void appendTcpTrajectoryPoint(
    std::vector<geometry_msgs::msg::Point>& trajectory,
    const geometry_msgs::msg::Point& point) {
    std::lock_guard<std::mutex> lock(tcp_trajectory_mutex_);
    if (!trajectory.empty()) {
      const auto& previous = trajectory.back();
      const double dx = point.x - previous.x;
      const double dy = point.y - previous.y;
      const double dz = point.z - previous.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < tcp_trajectory_min_distance_m_) {
        return;
      }
    }

    if (trajectory.size() >= tcp_trajectory_max_points_) {
      std::vector<geometry_msgs::msg::Point> reduced;
      reduced.reserve((trajectory.size() + 2) / 2);
      for (size_t i = 0; i < trajectory.size(); i += 2) {
        reduced.push_back(trajectory[i]);
      }
      if ((trajectory.size() - 1) % 2 != 0) {
        reduced.push_back(trajectory.back());
      }
      trajectory.swap(reduced);
    }
    trajectory.push_back(point);
  }

  void resetTcpTrajectories(const std::vector<double>& initial_joints) {
    {
      std::lock_guard<std::mutex> lock(tcp_trajectory_mutex_);
      planned_tcp_trajectory_.clear();
      measured_tcp_trajectory_.clear();
    }
    if (!publish_tcp_trajectory_) {
      record_tcp_trajectory_.store(false);
      return;
    }

    const auto initial_point = tcpPointForArmJoints(initial_joints);
    appendTcpTrajectoryPoint(planned_tcp_trajectory_, initial_point);
    appendTcpTrajectoryPoint(measured_tcp_trajectory_, initial_point);
    record_tcp_trajectory_.store(true);
  }

  void recordPlannedTcpTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory) {
    if (!publish_tcp_trajectory_ || !record_tcp_trajectory_.load() || trajectory.points.empty()) {
      return;
    }
    const auto& arm_joint_names = robot_model_->getPlanningGroup(arm_group_).joint_names;
    if (trajectory.joint_names != arm_joint_names) {
      return;
    }

    try {
      for (const auto& point : trajectory.points) {
        if (point.positions.size() != arm_joint_names.size()) {
          continue;
        }
        appendTcpTrajectoryPoint(
          planned_tcp_trajectory_, tcpPointForArmJoints(point.positions));
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Unable to record planned TCP trajectory: %s",
        e.what());
    }
  }

  void recordReplaySegment(
    const std::string& stage_name,
    const trajectory_msgs::msg::JointTrajectory& trajectory) {
    recordPlannedTcpTrajectory(trajectory);
    if (!replay_after_complete_ || trajectory.points.empty()) {
      return;
    }
    replay_segments_.push_back(ReplaySegment{stage_name, trajectory, e3_attached_to_tcp_});
  }

  trajectory_msgs::msg::JointTrajectory scaledTrajectoryForReplay(
    const trajectory_msgs::msg::JointTrajectory& trajectory) const {
    trajectory_msgs::msg::JointTrajectory out = trajectory;
    out.header.stamp = this->now();
    for (auto& point : out.points) {
      point.time_from_start =
        durationFromSec(pointTimeSec(point) / replay_speed_scale_);
    }
    return out;
  }

  double pointTimeSec(const trajectory_msgs::msg::JointTrajectoryPoint& point) const {
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) * 1e-9;
  }

  std::vector<double> currentNamedJointPositions(
    const std::vector<std::string>& joint_names) const {
    std::vector<double> positions;
    positions.reserve(joint_names.size());
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    for (const auto& joint_name : joint_names) {
      const auto arm_it =
        std::find(arm_group.joint_names.begin(), arm_group.joint_names.end(), joint_name);
      if (arm_it != arm_group.joint_names.end()) {
        positions.push_back(
          arm_state_[static_cast<size_t>(std::distance(arm_group.joint_names.begin(), arm_it))]);
        continue;
      }

      const auto gripper_it =
        std::find(gripper_joint_names_.begin(), gripper_joint_names_.end(), joint_name);
      if (gripper_it != gripper_joint_names_.end()) {
        positions.push_back(
          gripper_state_[static_cast<size_t>(std::distance(gripper_joint_names_.begin(), gripper_it))]);
        continue;
      }

      positions.push_back(0.0);
    }
    return positions;
  }

  void applyNamedJointPositions(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions) {
    const size_t n = std::min(joint_names.size(), positions.size());
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    for (size_t i = 0; i < n; ++i) {
      const auto arm_it =
        std::find(arm_group.joint_names.begin(), arm_group.joint_names.end(), joint_names[i]);
      if (arm_it != arm_group.joint_names.end()) {
        arm_state_[static_cast<size_t>(std::distance(arm_group.joint_names.begin(), arm_it))] =
          positions[i];
        continue;
      }

      const auto gripper_it =
        std::find(gripper_joint_names_.begin(), gripper_joint_names_.end(), joint_names[i]);
      if (gripper_it != gripper_joint_names_.end()) {
        gripper_state_[static_cast<size_t>(std::distance(gripper_joint_names_.begin(), gripper_it))] =
          positions[i];
      }
    }
  }

  std::vector<double> interpolatePositions(
    const std::vector<double>& from,
    const std::vector<double>& to,
    double ratio) const {
    const size_t n = std::min(from.size(), to.size());
    std::vector<double> out(n, 0.0);
    ratio = std::clamp(ratio, 0.0, 1.0);
    for (size_t i = 0; i < n; ++i) {
      out[i] = from[i] + (to[i] - from[i]) * ratio;
    }
    return out;
  }

  std::vector<double> sampleTrajectoryPositions(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const std::vector<double>& segment_start_positions,
    double elapsed_sec) const {
    if (trajectory.points.empty()) {
      return segment_start_positions;
    }

    if (trajectory.points.size() == 1) {
      const double duration = std::max(0.02, pointTimeSec(trajectory.points.front()));
      return interpolatePositions(
        segment_start_positions,
        trajectory.points.front().positions,
        elapsed_sec / duration);
    }

    if (elapsed_sec <= pointTimeSec(trajectory.points.front())) {
      return trajectory.points.front().positions;
    }

    for (size_t i = 1; i < trajectory.points.size(); ++i) {
      const double t0 = pointTimeSec(trajectory.points[i - 1]);
      const double t1 = pointTimeSec(trajectory.points[i]);
      if (elapsed_sec <= t1) {
        const double denom = std::max(1e-6, t1 - t0);
        return interpolatePositions(
          trajectory.points[i - 1].positions,
          trajectory.points[i].positions,
          (elapsed_sec - t0) / denom);
      }
    }

    return trajectory.points.back().positions;
  }

  std::vector<double> applyArmCommandVelocityLimit(
    const std::vector<double>& desired,
    const std::vector<double>& previous,
    double dt_sec) const {
    if (desired.size() != previous.size() ||
        desired.size() != max_arm_command_velocity_rad_s_.size()) {
      return desired;
    }

    std::vector<double> limited = desired;
    for (size_t i = 0; i < desired.size(); ++i) {
      const double vmax = max_arm_command_velocity_rad_s_[i];
      if (vmax <= 0.0) {
        continue;
      }
      const double max_step = vmax * std::max(1e-3, dt_sec);
      limited[i] = std::clamp(desired[i], previous[i] - max_step, previous[i] + max_step);
    }
    return limited;
  }

  void publishArmCommand(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions) {
    if (!arm_command_pub_ || joint_names.size() != positions.size()) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names;
    msg.position = positions;
    arm_command_pub_->publish(msg);
    arm_command_sent_.store(true);
  }

  void publishArmCommandCurrentState(const std::string& stage_name) {
    if (!arm_command_pub_) {
      return;
    }
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    publishArmCommand(group.joint_names, arm_state_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Stage [%s] published current arm command.",
      stage_name.c_str());
  }

  bool publishArmCommandTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const std::string& stage_name) {
    if (!arm_command_pub_ || trajectory.points.empty()) {
      RCLCPP_WARN(this->get_logger(), "[GRASP][command][SKIP] step=%s publisher=%s points=%zu",
        stage_name.c_str(), arm_command_pub_ ? "enabled" : "disabled", trajectory.points.size());
      return true;
    }

    const auto segment_start_positions = currentNamedJointPositions(trajectory.joint_names);
    std::vector<double> previous_command = segment_start_positions;
    const auto final_positions = trajectory.points.back().positions;
    const double original_duration = std::max(0.02, trajectoryDurationSec(trajectory));
    const double period_sec = 1.0 / arm_command_publish_hz_;
    const auto started_at = std::chrono::steady_clock::now();
    auto previous_tick = started_at;

    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][command][BEGIN] step=%s hz=%.2f speed_scale=%.3f duration=%.3fs",
      stage_name.c_str(),
      arm_command_publish_hz_,
      arm_command_speed_scale_,
      original_duration);

    publishArmCommand(trajectory.joint_names, previous_command);

    while (!stopping()) {
      const auto now_tp = std::chrono::steady_clock::now();
      const double wall_elapsed = std::chrono::duration<double>(now_tp - started_at).count();
      const double dt_sec = std::max(1e-3, std::chrono::duration<double>(now_tp - previous_tick).count());
      previous_tick = now_tp;

      const double trajectory_elapsed =
        std::min(original_duration, wall_elapsed * arm_command_speed_scale_);
      const auto desired = sampleTrajectoryPositions(
        trajectory,
        segment_start_positions,
        trajectory_elapsed);
      previous_command = applyArmCommandVelocityLimit(desired, previous_command, dt_sec);
      publishArmCommand(trajectory.joint_names, previous_command);

      if (trajectory_elapsed >= original_duration &&
          maxJointError(previous_command, final_positions) <= arm_command_goal_tolerance_rad_) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(period_sec));
    }

    // An aborted stream must hold where the arm was told to be, not jump to
    // the final target.
    const bool aborted = stopping();
    publishArmCommand(trajectory.joint_names, aborted ? previous_command : final_positions);
    if (aborted) {
      applyNamedJointPositions(trajectory.joint_names, previous_command);
      logArmJointState(stage_name + "_abort_hold_command", previous_command);
      if (emergency_abort_.load()) {
        publishMeasuredHold(stage_name + "_emergency_hold");
      }
    }
    RCLCPP_INFO(this->get_logger(), "[GRASP][command][END] step=%s status=%s subscribers=%zu (publish is not hardware acknowledgement)",
      stage_name.c_str(), aborted ? "ABORTED_HOLD" : "SENT",
      arm_command_pub_->get_subscription_count());
    return !aborted;
  }

  bool executeArmJointPlan(
    const std::vector<double>& target_joints,
    const std::string& stage_name) {
    RCLCPP_INFO(this->get_logger(), "[GRASP][plan][BEGIN] step=%s goal=joint service=plan_arm_motion",
      stage_name.c_str());
    const auto planning_start_joints =
      clampArmJointsToLimits(arm_state_, stage_name, "start");
    const auto planning_target_joints =
      clampArmJointsToLimits(target_joints, stage_name, "goal");
    if (planning_start_joints != arm_state_) {
      arm_state_ = planning_start_joints;
      publishCurrentTheoreticalJointState(stage_name + "_clamped_start_sync");
    }

    if (planning_target_joints.size() == arm_state_.size()) {
      double max_err = 0.0;
      for (size_t i = 0; i < planning_target_joints.size(); ++i) {
        max_err = std::max(max_err, std::abs(planning_target_joints[i] - arm_state_[i]));
      }
      if (max_err < 1e-6) {
        RCLCPP_INFO(this->get_logger(), "Stage [%s] already at target arm joints.", stage_name.c_str());
        const auto command_started = std::chrono::steady_clock::now();
    snapshotMeasuredArmStateForMotionCheck();
        publishArmCommandCurrentState(stage_name);
        if (!publishGroupHoldTrajectory(arm_group_, planning_target_joints, stage_name, true)) {
          return false;
        }
        if (!waitForArmAtTarget(planning_target_joints, stage_name, command_started)) {
          return false;
        }
        publishCurrentTheoreticalJointState(stage_name + "_skip_sync");
        sleepAfterAction(post_action_wait_sec_);
        return true;
      }
    }

    auto req = std::make_shared<lite_motion_msgs::srv::PlanArmMotion::Request>();
    req->group_name = arm_group_;
    req->ee_frame = tcp_frame_;
    req->use_pose_goal = false;
    req->start_joint_positions = planning_start_joints;
    req->goal_joint_positions = planning_target_joints;
    req->plan_only = false;
    req->use_cartesian_path = false;
    req->cartesian_max_step = 0.0;
    req->use_request_solver_settings = true;
    req->request_planning_timeout = joint_planning_timeout_;
    req->request_path_check_step = path_check_step_m_;

    auto resp = callService<lite_motion_msgs::srv::PlanArmMotion>(
      plan_client_, req, "plan_arm_motion");
    if (!resp || !resp->success) {
      last_plan_failure_ = resp ? resp->message : "no response";
      RCLCPP_ERROR(
        this->get_logger(),
        "[GRASP][plan][FAIL] step=%s reason=%s; arm_command_not_sent",
        stage_name.c_str(),
        last_plan_failure_.c_str());
      return false;
    }
    if (resp->trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Arm joint planning returned empty trajectory at stage [%s]", stage_name.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "[GRASP][plan][OK] step=%s points=%zu",
      stage_name.c_str(), resp->trajectory.points.size());
    recordReplaySegment(stage_name, resp->trajectory);
    const auto final_arm_state = resp->trajectory.points.back().positions;
    const double target_err = maxJointError(final_arm_state, planning_target_joints);
    logArmJointState(stage_name + "_planned_final", final_arm_state);
    if (stage_name == "move_arm_to_observe_before_perception") {
      // 记下"规划出来的"观测位, 冻结相机帧时用它, 保证每次运行可复现。
      observed_reference_joints_ = final_arm_state;
    }
    if (target_err > joint_goal_tolerance_rad_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Stage [%s] final joint state is %.6f rad away from requested target, tolerance %.6f rad.",
        stage_name.c_str(),
        target_err,
        joint_goal_tolerance_rad_);
      return false;
    }
    const auto command_started = std::chrono::steady_clock::now();
    snapshotMeasuredArmStateForMotionCheck();
    if (arm_command_pub_) {
      if (!publishArmCommandTrajectory(resp->trajectory, stage_name)) {
        return false;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "[GRASP][command][SKIP] step=%s arm_command_publisher=disabled; waiting only",
        stage_name.c_str());
      sleepAfterAction(effectiveExecutionWaitSec(resp->trajectory));
    }
    if (!waitForArmAtTarget(final_arm_state, stage_name, command_started)) {
      return false;
    }
    if (!publishGroupHoldTrajectory(
          arm_group_, final_arm_state, stage_name + "_endpoint_hold")) {
      return false;
    }
    publishCurrentTheoreticalJointState(stage_name + "_final_sync");
    return true;
  }

  // A single joint-space goal from the wall-side grasp pose to the rear place
  // pose turns the base joint by about half a revolution. The base joint is the
  // slowest one, so a direct plan keeps the arm extended (~0.47 m) while it
  // sweeps the whole arc, which clips side walls when the rover parks next to
  // one. Move through the compact carry pose instead: fold with the base joint
  // held, rotate the base joint while folded, then extend at the destination.
  // Any failed stage falls back to the original direct plan.
  bool executeArmStagedYawPlan(
    const std::vector<double>& target_joints,
    const std::string& stage_name) {
    const auto direct_plan = [&]() {
        return executeArmJointPlan(target_joints, stage_name);
      };
    if (!staged_yaw_enabled_ ||
      target_joints.size() != arm_state_.size() ||
      carry_tuck_joint_positions_.size() != arm_state_.size()) {
      return direct_plan();
    }
    const double yaw_delta = std::abs(target_joints.front() - arm_state_.front());
    if (yaw_delta < staged_yaw_min_rad_) {
      return direct_plan();
    }

    std::vector<double> tuck_start = carry_tuck_joint_positions_;
    std::vector<double> tuck_goal = carry_tuck_joint_positions_;
    tuck_start.front() = arm_state_.front();
    tuck_goal.front() = target_joints.front();
    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][retract][STAGED] step=%s yaw_delta=%.3frad from=%.3f to=%.3f; tuck -> rotate_folded -> extend",
      stage_name.c_str(), yaw_delta, arm_state_.front(), target_joints.front());
    // A failed stage leaves arm_state_ at the pre-stage pose while the arm is
    // part-way; the fallback must start from fresh measured joints, and the
    // unattended shutdown/cancel recovery gets no fallback sweep at all.
    const auto fallback = [&](const char* stage) {
        if (stopping()) {
          return false;  // aborted: no fallback motion
        }
        if (restricted_recovery_.load()) {
          RCLCPP_ERROR(
            this->get_logger(),
            "[GRASP][retract][NO_FALLBACK] step=%s stage=%s failed; direct joint sweep disabled during shutdown/cancel recovery.",
            stage_name.c_str(), stage);
          return false;
        }
        RCLCPP_WARN(
          this->get_logger(),
          "[GRASP][retract][FALLBACK] step=%s stage=%s; refreshing the measured start state and using the direct joint plan.",
          stage_name.c_str(), stage);
        if (!readInitialArmJointPositions(arm_state_)) {
          return false;
        }
        return direct_plan();
      };
    if (!executeArmJointPlan(tuck_start, stage_name + "_tuck")) {
      return fallback("tuck");
    }
    if (!executeArmJointPlan(tuck_goal, stage_name + "_rotate_folded")) {
      return fallback("rotate_folded");
    }
    if (!executeArmJointPlan(target_joints, stage_name + "_extend")) {
      return fallback("extend");
    }
    return true;
  }

  // Face yaw of a target with a square cross-section (see tzb.target_yaw_symmetry_deg).
  double symmetricFaceYaw(const Eigen::Matrix3d& rot, double x, double y) const {
    int vertical = 0;
    for (int k = 1; k < 3; ++k) {
      if (std::abs(rot(2, k)) > std::abs(rot(2, vertical))) vertical = k;
    }
    const int horizontal = (vertical + 1) % 3;
    const double face = std::atan2(rot(1, horizontal), rot(0, horizontal));
    const double step = target_yaw_symmetry_deg_ * M_PI / 180.0;
    // grasp_yaw_offset_deg_ is added after this function; aim so the final yaw is near radial.
    const double reference = std::atan2(y, x) +
      (target_yaw_reference_offset_deg_ - grasp_yaw_offset_deg_) * M_PI / 180.0;
    double best = face;
    double best_err = std::numeric_limits<double>::infinity();
    const int n = std::max(1, static_cast<int>(std::lround(2.0 * M_PI / step)));
    for (int i = 0; i < n; ++i) {
      const double cand = face + i * step;
      const double err = std::abs(std::remainder(cand - reference, 2.0 * M_PI));
      if (err < best_err) { best_err = err; best = cand; }
    }
    return std::remainder(best, 2.0 * M_PI);
  }

  std::vector<geometry_msgs::msg::PoseStamped> buildAdaptiveRpyTargets(
    const geometry_msgs::msg::PoseStamped& target_pose) const {
    std::vector<double> offset_levels{0.0};
    for (double offset = adaptive_rpy_step_rad_; offset <= adaptive_rpy_max_rad_ + 1e-9;
      offset += adaptive_rpy_step_rad_)
    {
      offset_levels.push_back(offset);
      offset_levels.push_back(-offset);
    }
    if (adaptive_rpy_max_rad_ > 1e-9 &&
      std::abs(offset_levels.back()) < adaptive_rpy_max_rad_ - 1e-9)
    {
      offset_levels.push_back(adaptive_rpy_max_rad_);
      offset_levels.push_back(-adaptive_rpy_max_rad_);
    }

    std::vector<std::array<double, 3>> offsets;
    offsets.reserve(offset_levels.size() * offset_levels.size() * offset_levels.size());
    for (const double roll : offset_levels) {
      for (const double pitch : offset_levels) {
        for (const double yaw : offset_levels) {
          if (std::abs(roll) < 1e-12 && std::abs(pitch) < 1e-12 &&
            std::abs(yaw) < 1e-12)
          {
            continue;
          }
          offsets.push_back({roll, pitch, yaw});
        }
      }
    }

    std::sort(
      offsets.begin(), offsets.end(), [](const std::array<double, 3>& lhs,
      const std::array<double, 3>& rhs) {
        const double lhs_max = std::max({std::abs(lhs[0]), std::abs(lhs[1]), std::abs(lhs[2])});
        const double rhs_max = std::max({std::abs(rhs[0]), std::abs(rhs[1]), std::abs(rhs[2])});
        if (std::abs(lhs_max - rhs_max) > 1e-12) {
          return lhs_max < rhs_max;
        }
        const int lhs_nonzero =
          static_cast<int>(std::abs(lhs[0]) > 1e-12) +
          static_cast<int>(std::abs(lhs[1]) > 1e-12) +
          static_cast<int>(std::abs(lhs[2]) > 1e-12);
        const int rhs_nonzero =
          static_cast<int>(std::abs(rhs[0]) > 1e-12) +
          static_cast<int>(std::abs(rhs[1]) > 1e-12) +
          static_cast<int>(std::abs(rhs[2]) > 1e-12);
        if (lhs_nonzero != rhs_nonzero) {
          return lhs_nonzero > rhs_nonzero;
        }
        const double lhs_sum = std::abs(lhs[0]) + std::abs(lhs[1]) + std::abs(lhs[2]);
        const double rhs_sum = std::abs(rhs[0]) + std::abs(rhs[1]) + std::abs(rhs[2]);
        if (std::abs(lhs_sum - rhs_sum) > 1e-12) {
          return lhs_sum < rhs_sum;
        }
        return lhs < rhs;
      });

    const size_t max_offset_count = static_cast<size_t>(std::max(0, adaptive_rpy_max_candidates_ - 1));
    if (offsets.size() > max_offset_count) {
      offsets.resize(max_offset_count);
    }

    const Eigen::Quaterniond original_q = quaternionEigenFromMsg(target_pose.pose.orientation);
    std::vector<geometry_msgs::msg::PoseStamped> targets;
    targets.reserve(offsets.size() + 1);
    targets.push_back(target_pose);
    for (const auto& offset : offsets) {
      const Eigen::Quaterniond delta =
        Eigen::AngleAxisd(offset[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(offset[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(offset[0], Eigen::Vector3d::UnitX());
      const Eigen::Quaterniond candidate_q = (delta * original_q).normalized();

      geometry_msgs::msg::PoseStamped candidate = target_pose;
      candidate.pose.orientation.x = candidate_q.x();
      candidate.pose.orientation.y = candidate_q.y();
      candidate.pose.orientation.z = candidate_q.z();
      candidate.pose.orientation.w = candidate_q.w();
      targets.push_back(std::move(candidate));
    }
    return targets;
  }

  // precise_arrival: after the normal arrival confirmation run the precise
  // pre-grasp arrival (stop + residual compensation + tight band); used by the
  // grasp-approach stage only.
  bool executeArmPosePlan(
    const geometry_msgs::msg::PoseStamped& target_pose,
    bool use_cartesian_path,
    const std::string& stage_name,
    bool precise_arrival = false) {
    RCLCPP_INFO(this->get_logger(),
      "[GRASP][plan][BEGIN] step=%s goal=pose service=plan_arm_motion response_timeout=%.2fs",
      stage_name.c_str(), response_wait_sec_);
    const auto candidate_targets = adaptive_rpy_ ?
      buildAdaptiveRpyTargets(target_pose) :
      std::vector<geometry_msgs::msg::PoseStamped>{target_pose};
    const Eigen::Vector3d original_zyx =
      quaternionEigenFromMsg(target_pose.pose.orientation).toRotationMatrix().eulerAngles(2, 1, 0);
    const Eigen::Vector3d original_rpy(original_zyx.z(), original_zyx.y(), original_zyx.x());
    if (adaptive_rpy_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Adaptive RPY planning at fixed xyz=(%.4f, %.4f, %.4f) in [%s], centered on transformed TCP RPY=(%.4f, %.4f, %.4f) rad; candidates=%zu, step=%.2f deg, max=%.2f deg.",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        planning_frame_.c_str(),
        original_rpy.x(),
        original_rpy.y(),
        original_rpy.z(),
        candidate_targets.size(),
        adaptive_rpy_step_rad_ * 180.0 / kPi,
        adaptive_rpy_max_rad_ * 180.0 / kPi);
    }

    trajectory_msgs::msg::JointTrajectory selected_trajectory;
    size_t selected_candidate = 0;
    for (size_t candidate_index = 0; candidate_index < candidate_targets.size(); ++candidate_index) {
      const auto& candidate_target = candidate_targets[candidate_index];
      auto req = std::make_shared<lite_motion_msgs::srv::PlanArmMotion::Request>();
      req->group_name = arm_group_;
      req->ee_frame = tcp_frame_;
      req->use_pose_goal = true;
      req->target_pose = candidate_target;
      req->start_joint_positions = arm_state_;
      req->plan_only = adaptive_rpy_;
      req->use_cartesian_path = use_cartesian_path;
      req->cartesian_max_step = use_cartesian_path ? cartesian_max_step_m_ : 0.0;
      req->use_weighted_orientation_goal = position_only_target_;
      req->orientation_weight_x = 0.0;
      req->orientation_weight_y = 0.0;
      req->orientation_weight_z = 0.0;

      if (use_cartesian_path && cartesian_use_request_solver_settings_) {
        req->use_request_solver_settings = true;
        req->request_ik_max_iterations = cartesian_ik_max_iterations_;
        req->request_ik_pos_tolerance = cartesian_ik_pos_tolerance_;
        req->request_ik_rot_tolerance = cartesian_ik_rot_tolerance_;
        req->request_ik_damping = cartesian_ik_damping_;
        req->request_ik_alpha = cartesian_ik_alpha_;
        req->request_ik_max_step_norm = cartesian_ik_max_step_norm_;
        req->request_ik_max_seed_count = cartesian_ik_max_seed_count_;
        req->request_planning_timeout = cartesian_planning_timeout_;
        req->request_path_check_step = path_check_step_m_;
        req->request_use_sdls = cartesian_use_sdls_;
        req->request_cartesian_nullspace_gain = cartesian_nullspace_gain_;
        req->request_cartesian_singular_threshold = cartesian_singular_threshold_;
        req->request_cartesian_branch_jump_max_rad = cartesian_branch_jump_max_rad_;
        req->request_cartesian_branch_jump_norm_max = cartesian_branch_jump_norm_max_;
        req->request_cartesian_waypoint_pos_tolerance = cartesian_waypoint_pos_tolerance_;
        req->request_cartesian_waypoint_rot_tolerance = cartesian_waypoint_rot_tolerance_;
        req->request_cartesian_min_interpolation_steps = cartesian_min_interpolation_steps_;
        req->request_cartesian_refinement_max_steps = cartesian_refinement_max_steps_;
      } else {
        req->use_request_solver_settings = true;
        req->request_planning_timeout = joint_planning_timeout_;
        req->request_path_check_step = path_check_step_m_;
      }

      auto resp = callService<lite_motion_msgs::srv::PlanArmMotion>(
        plan_client_, req, "plan_arm_motion");
      if (!resp || !resp->success || resp->trajectory.points.empty()) {
        const std::string reason = !resp ? "no response" :
          (!resp->success ? resp->message : "empty trajectory");
        if (!adaptive_rpy_) {
          last_plan_failure_ = reason;
          RCLCPP_ERROR(
            this->get_logger(),
            "[GRASP][plan][FAIL] step=%s reason=%s; arm_command_not_sent",
            stage_name.c_str(), reason.c_str());
          return false;
        }
        RCLCPP_WARN(
          this->get_logger(),
          "Adaptive RPY candidate %zu/%zu failed at stage [%s]: %s",
          candidate_index + 1,
          candidate_targets.size(),
          stage_name.c_str(),
          reason.c_str());
        continue;
      }

      selected_trajectory = resp->trajectory;
      selected_candidate = candidate_index;
      break;
    }

    if (selected_trajectory.points.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Arm pose planning failed at stage [%s]: no adaptive RPY candidate produced a trajectory.",
        stage_name.c_str());
      return false;
    }
    if (adaptive_rpy_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Adaptive RPY selected candidate %zu/%zu for stage [%s]; TCP xyz remains unchanged.",
        selected_candidate + 1,
        candidate_targets.size(),
        stage_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "[GRASP][plan][OK] step=%s points=%zu",
      stage_name.c_str(), selected_trajectory.points.size());
    recordReplaySegment(stage_name, selected_trajectory);
    const auto final_arm_state = selected_trajectory.points.back().positions;
    logArmJointState(stage_name + "_planned_final", final_arm_state);
    const auto command_started = std::chrono::steady_clock::now();
    snapshotMeasuredArmStateForMotionCheck();
    if (arm_command_pub_) {
      if (!publishArmCommandTrajectory(selected_trajectory, stage_name)) {
        return false;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "[GRASP][command][SKIP] step=%s arm_command_publisher=disabled; waiting only",
        stage_name.c_str());
      sleepAfterAction(effectiveExecutionWaitSec(selected_trajectory));
    }
    if (!waitForArmAtTarget(final_arm_state, stage_name, command_started)) {
      return false;
    }
    if (precise_arrival && !preciseGraspArrival(final_arm_state, stage_name)) {
      return false;
    }
    if (!publishGroupHoldTrajectory(
          arm_group_, final_arm_state, stage_name + "_endpoint_hold")) {
      return false;
    }
    publishCurrentTheoreticalJointState(stage_name + "_final_sync");
    return true;
  }

  // Measured joints at the moment a command stream starts; used to tell a
  // joint that never moved (lower controller / enable problem) from one that
  // is merely slow or short of the tolerance.
  void snapshotMeasuredArmStateForMotionCheck() {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    motion_start_measured_ = have_complete_arm_state_ ? latest_measured_arm_positions_ : std::vector<double>{};
  }

  // Largest commanded travel of a stage (target vs the measured joints at the
  // moment the command stream started) and the largest measured change since
  // then. Precondition: arm_state_mutex_ held by the caller.
  struct StageMotionSummary {
    double max_travel{0.0};
    size_t max_travel_joint{0};
    double max_moved{0.0};
    size_t commanded_joints{0};  // joints whose commanded travel exceeds feedback noise
    bool valid{false};
  };
  StageMotionSummary summarizeStageMotion(
    const std::vector<double>& target,
    const std::vector<double>& measured) const {
    StageMotionSummary s;
    if (target.empty() || target.size() != measured.size() ||
        target.size() != motion_start_measured_.size()) {
      return s;
    }
    s.valid = true;
    for (size_t i = 0; i < target.size(); ++i) {
      const double travel = std::abs(std::remainder(target[i] - motion_start_measured_[i], 2.0 * kPi));
      const double moved = std::abs(std::remainder(measured[i] - motion_start_measured_[i], 2.0 * kPi));
      if (travel > s.max_travel) {
        s.max_travel = travel;
        s.max_travel_joint = i;
      }
      if (travel > kNoMotionNoiseRad) {
        ++s.commanded_joints;
      }
      s.max_moved = std::max(s.max_moved, moved);
    }
    return s;
  }

  // Classifies a stage that ended outside its tolerance:
  //   NO_MOTION          commands enabled, largest commanded travel >= 0.02 rad
  //                      and NO joint left the feedback noise band (0.002 rad):
  //                      the lower controller executed nothing (field 11-06
  //                      18:27: remote controller off; feedback keeps streaming)
  //   STUCK              a joint with commanded travel >= 3 x tolerance did not
  //                      move while the arm as a whole did
  //   DEADBAND_OR_OFFSET a joint with commanded travel <  3 x tolerance did not
  //                      move while the arm as a whole did (field J2 case:
  //                      0.10007 vs tolerance 0.10 with J5/J6 executing)
  //   FEEDBACK_STALE     the last sample is older than arm_state_max_age_sec
  // Per joint, "no motion" = measured change < max(0.01 rad, 10 % of the
  // commanded travel).
  // Precondition: the caller (waitForArmAtTarget) holds arm_state_mutex_; this
  // function must not lock it again (non-recursive mutex, executor callback
  // onArmState would then block on it).
  void reportStuckJoints(
    const std::string& stage_name,
    const std::vector<double>& target,
    const std::vector<double>& measured) {
    bool fresh = false;
    double age = std::numeric_limits<double>::infinity();
    if (have_complete_arm_state_) {
      age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_arm_state_time_).count();
      fresh = age <= arm_state_max_age_sec_;
    }
    if (!fresh) {
      last_feedback_stale_ = true;
      RCLCPP_ERROR(
        this->get_logger(),
        "[GRASP][feedback][FEEDBACK_STALE] step=%s last feedback sample is %.2fs old (max %.2fs); joint motion cannot be classified.",
        stage_name.c_str(), age, arm_state_max_age_sec_);
      return;
    }
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    const auto motion = summarizeStageMotion(target, measured);
    if (arm_command_pub_ && motion.valid && motion.max_travel >= kNoMotionMinTravelRad &&
        motion.max_moved < kNoMotionNoiseRad && motion.max_travel_joint < group.joint_names.size()) {
      last_no_motion_ = true;
      const auto& joint = group.joint_names[motion.max_travel_joint];
      std::string single_joint_note;
      if (motion.commanded_joints == 1) {
        single_joint_note =
          " (only " + joint + " was commanded to move: a blocked joint or a per-joint deadband/offset"
          " cannot be told apart from a disabled controller in this stage)";
      }
      RCLCPP_ERROR(
        this->get_logger(),
        "[GRASP][feedback][NO_MOTION] step=%s lower controller did not execute any joint command (commanded travel up to %.3f rad on %s; largest measured change %.4f rad < %.3f); %s%s",
        stage_name.c_str(), motion.max_travel, joint.c_str(), motion.max_moved, kNoMotionNoiseRad,
        kNoMotionAdvice, single_joint_note.c_str());
      return;
    }
    const size_t n = std::min({target.size(), measured.size(), motion_start_measured_.size(), group.joint_names.size()});
    std::vector<std::string> stuck_names;
    std::vector<std::string> deadband_names;
    for (size_t i = 0; i < n; ++i) {
      const double tol = jointToleranceRad(group.joint_names[i]);
      const double commanded_travel = std::remainder(target[i] - motion_start_measured_[i], 2.0 * kPi);
      const double moved = std::remainder(measured[i] - motion_start_measured_[i], 2.0 * kPi);
      const double error = std::remainder(measured[i] - target[i], 2.0 * kPi);
      if (std::abs(error) <= tol || std::abs(commanded_travel) <= tol ||
          std::abs(moved) >= std::max(0.01, 0.1 * std::abs(commanded_travel))) {
        continue;
      }
      if (std::abs(commanded_travel) >= 3.0 * tol) {
        stuck_names.push_back(group.joint_names[i]);
        RCLCPP_ERROR(
          this->get_logger(),
          "[GRASP][feedback][STUCK] step=%s joint=%s commanded=%.5f start=%.5f measured=%.5f moved=%.5f error=%.5f tolerance=%.3f: no motion observed for a %.3f rad command; check the lower controller / joint enable / mechanical blockage before retrying.",
          stage_name.c_str(), group.joint_names[i].c_str(), target[i], motion_start_measured_[i], measured[i],
          moved, error, tol, std::abs(commanded_travel));
      } else {
        deadband_names.push_back(group.joint_names[i]);
        RCLCPP_ERROR(
          this->get_logger(),
          "[GRASP][feedback][DEADBAND_OR_OFFSET] joint=%s commanded=%.5f measured=%.5f residual=%.5f (tolerance %.3f, commanded travel %.5f < 3x tolerance): the joint did not move for a small command; likely a lower-controller deadband or a calibration offset, not a blocked joint. step=%s",
          group.joint_names[i].c_str(), target[i], measured[i], std::abs(error) - tol, tol,
          std::abs(commanded_travel), stage_name.c_str());
      }
    }
    const auto join = [](const std::vector<std::string>& names) {
        std::string joined;
        for (const auto& name : names) {
          joined += (joined.empty() ? "" : ",") + name;
        }
        return joined;
      };
    last_stuck_joints_ = join(stuck_names);
    last_deadband_joints_ = join(deadband_names);
  }

  bool readInitialArmJointPositions(std::vector<double>& positions) {
    if (!arm_feedback_enabled_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Arm feedback is disabled; cannot determine a safe planning start state without serial feedback.");
      return false;
    }
    std::unique_lock<std::mutex> lock(arm_state_mutex_);
    const auto is_recent = [this]() {
      return have_complete_arm_state_ &&
             std::chrono::duration<double>(
               std::chrono::steady_clock::now() - latest_arm_state_time_).count() <=
               arm_state_max_age_sec_;
    };
    if (!is_recent()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting up to %.2f s for a complete arm state on [%s].",
        arm_state_wait_timeout_sec_,
        arm_state_topic_.c_str());
      const auto ready = [this, &is_recent]() {
        return stopping() || is_recent();
      };
      if (arm_state_wait_timeout_sec_ <= 0.0) {
        arm_state_cv_.wait(lock, ready);
      } else {
        arm_state_cv_.wait_for(
          lock, std::chrono::duration<double>(arm_state_wait_timeout_sec_), ready);
      }
    }
    if (!stopping() && is_recent()) {
      const auto measured = latest_measured_arm_positions_;
      lock.unlock();
      logArmJointState("serial_initial_state", measured);
      positions = measured;
      return true;
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "No complete recent arm state received within %.2f s on [%s]; refusing to plan from a fabricated arm state.",
      arm_state_wait_timeout_sec_,
      arm_state_topic_.c_str());
    return false;
  }

  bool waitForArmAtTarget(
    const std::vector<double>& target,
    const std::string& stage_name,
    const std::chrono::steady_clock::time_point& command_started) {
    RCLCPP_INFO(this->get_logger(),
      "[GRASP][feedback][WAIT] step=%s enabled=%s topic=%s timeout=%.2fs tolerance=%.4frad stable=%.2fs",
      stage_name.c_str(), arm_feedback_enabled_ ? "true" : "false", arm_state_topic_.c_str(),
      arm_motion_feedback_timeout_sec_, arm_motion_goal_tolerance_rad_, arm_motion_stable_duration_sec_);
    if (target.empty()) {
      return false;
    }
    last_stuck_joints_.clear();
    last_deadband_joints_.clear();
    last_feedback_stale_ = false;
    last_no_motion_ = false;
    if (!arm_feedback_enabled_) {
      arm_state_ = target;
      return true;
    }

    // Command sampling is velocity-limited and can itself exceed the feedback timeout.
    // Start the arrival window after the final command has been sent, while still
    // accepting feedback received during command publication.
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(arm_motion_feedback_timeout_sec_));
    std::unique_lock<std::mutex> lock(arm_state_mutex_);
    uint64_t observed_sequence = arm_state_sequence_;
    bool stable = false;
    std::chrono::steady_clock::time_point stable_since{};
    double last_error = std::numeric_limits<double>::infinity();
    std::vector<double> last_measured_positions;
    bool no_motion_warned = false;

    const auto evaluate_latest_feedback = [this, &target, &command_started, &stable,
                                           &stable_since, &last_error,
                                           &last_measured_positions, &stage_name,
                                           &no_motion_warned]() {
      if (!have_complete_arm_state_ || latest_arm_state_time_ < command_started ||
          latest_measured_arm_positions_.size() != target.size()) {
        return false;
      }
      last_measured_positions = latest_measured_arm_positions_;
      last_error = maxJointError(latest_measured_arm_positions_, target);
      // Early warning (once): a streamed command and still no joint outside
      // the feedback noise band after no_motion_warning_sec. The wait goes on
      // until the normal timeout; the operator may switch the remote on.
      if (!no_motion_warned && arm_command_pub_ && no_motion_warning_sec_ > 0.0 &&
          std::chrono::duration<double>(latest_arm_state_time_ - command_started).count() >=
            no_motion_warning_sec_) {
        const auto motion = summarizeStageMotion(target, latest_measured_arm_positions_);
        if (motion.valid && motion.max_travel >= kNoMotionEarlyWarnTravelRad &&
            motion.max_moved < kNoMotionNoiseRad) {
          no_motion_warned = true;
          const auto& group = robot_model_->getPlanningGroup(arm_group_);
          RCLCPP_WARN(
            this->get_logger(),
            "[GRASP][feedback][NO_MOTION][EARLY] step=%s no joint has moved %.1fs after the command (commanded travel up to %.3f rad on %s); %s; still waiting up to %.1fs for arrival",
            stage_name.c_str(), no_motion_warning_sec_, motion.max_travel,
            motion.max_travel_joint < group.joint_names.size() ? group.joint_names[motion.max_travel_joint].c_str() : "?",
            kNoMotionAdvice, arm_motion_feedback_timeout_sec_);
        }
      }
      const double normalized_error =
        maxNormalizedJointError(latest_measured_arm_positions_, target);
      const auto feedback_time = latest_arm_state_time_;
      if (normalized_error <= 1.0) {
        if (!stable) {
          stable = true;
          stable_since = feedback_time;
        }
        if (std::chrono::duration<double>(feedback_time - stable_since).count() >=
            arm_motion_stable_duration_sec_) {
          arm_state_ = latest_measured_arm_positions_;
          RCLCPP_INFO(
            this->get_logger(),
            "[GRASP][feedback][REACHED] step=%s max_error=%.5frad stable=%.2fs.",
            stage_name.c_str(),
            last_error,
            arm_motion_stable_duration_sec_);
          return true;
        }
      } else {
        stable = false;
      }
      return false;
    };

    while (!stopping()) {
      if (evaluate_latest_feedback()) {
        return true;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      arm_state_cv_.wait_until(lock, deadline, [this, &observed_sequence]() {
        return stopping() || arm_state_sequence_ != observed_sequence;
      });
      if (stopping()) {
        return false;
      }
      if (arm_state_sequence_ == observed_sequence) {
        break;
      }
      observed_sequence = arm_state_sequence_;
    }
    if (stopping()) {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRASP][feedback][ABORT] step=%s arrival wait abandoned (last max error %.5f rad)",
        stage_name.c_str(), last_error);
      return false;
    }

    if (require_arm_feedback_for_completion_) {
      if (!std::isfinite(last_error)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Stage [%s] received no complete post-command arm feedback on [%s] within %.2f s; expected all joints in planning group [%s]. Aborting before the next subtask.",
          stage_name.c_str(),
          arm_state_topic_.c_str(),
          arm_motion_feedback_timeout_sec_,
          arm_group_.c_str());
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Stage [%s] did not receive physical-arrival confirmation within %.2f s (last max error %.5f rad); joint_errors:%s; aborting before the next subtask.",
          stage_name.c_str(),
          arm_motion_feedback_timeout_sec_,
          last_error,
          formatArmJointErrors(last_measured_positions, target).c_str());
        reportStuckJoints(stage_name, target, last_measured_positions);
      }
      return false;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "Stage [%s] physical-arrival confirmation timed out; last max error %.5f rad; joint_errors:%s; continuing because tzb.require_arm_feedback_for_completion=false.",
      stage_name.c_str(),
      last_error,
      formatArmJointErrors(last_measured_positions, target).c_str());
    arm_state_ = target;
    return true;
  }

  // ---- precise pre-grasp arrival ------------------------------------------

  double graspArrivalToleranceRad(const std::string& joint_name) const {
    if (joint_name == "J6_joint" || joint_name == "J6_Link" || joint_name == "J6_link") {
      return grasp_arrival_tolerance_j6_rad_;
    }
    return grasp_arrival_tolerance_rad_;
  }

  std::string formatGraspResiduals(
    const std::vector<double>& measured,
    const std::vector<double>& target) const {
    std::ostringstream oss;
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    const size_t n = std::min({measured.size(), target.size(), group.joint_names.size()});
    for (size_t i = 0; i < n; ++i) {
      const double error = std::remainder(target[i] - measured[i], 2.0 * kPi);
      const double tol = graspArrivalToleranceRad(group.joint_names[i]);
      oss << " " << group.joint_names[i]
          << "{target=" << target[i] << ", measured=" << measured[i]
          << ", residual=" << error << ", tol=" << tol
          << (std::abs(error) > tol ? ", BAD}" : ", ok}");
    }
    return oss.str();
  }

  enum class StopWait { STOPPED, TIMEOUT, ABORTED, NO_FEEDBACK };

  // Blocks until the arm has physically stopped: over the last
  // grasp_arrival_stable_sec of feedback every joint's position range divided
  // by the window length is <= grasp_arrival_max_speed_rad_s (0.02 rad/s over
  // 0.5 s = 0.01 rad of travel; the feedback quantisation is ~0.001 rad). Only
  // samples received after the call count. Returns TIMEOUT at the deadline,
  // ABORTED on shutdown/cancel. measured = the last sample, max_speed/span =
  // the worst joint's range/window of the last evaluated window.
  StopWait waitForArmStopped(
    const std::chrono::steady_clock::time_point& deadline,
    std::vector<double>& measured,
    double& max_speed_rad_s,
    double& span_sec) {
    const double window = grasp_arrival_stable_sec_;
    const auto started = std::chrono::steady_clock::now();
    std::deque<std::pair<std::chrono::steady_clock::time_point, std::vector<double>>> samples;
    max_speed_rad_s = std::numeric_limits<double>::infinity();
    span_sec = 0.0;
    std::unique_lock<std::mutex> lock(arm_state_mutex_);
    uint64_t observed_sequence = arm_state_sequence_;
    while (!stopping()) {
      arm_state_cv_.wait_until(lock, deadline, [this, &observed_sequence]() {
        return stopping() || arm_state_sequence_ != observed_sequence;
      });
      if (stopping()) {
        break;
      }
      if (arm_state_sequence_ != observed_sequence) {
        observed_sequence = arm_state_sequence_;
        if (have_complete_arm_state_ && latest_arm_state_time_ >= started) {
          samples.emplace_back(latest_arm_state_time_, latest_measured_arm_positions_);
          // Keep the oldest sample that still makes the window span >= window.
          while (samples.size() >= 2 &&
                 std::chrono::duration<double>(samples.back().first - samples[1].first).count() >= window) {
            samples.pop_front();
          }
          const double span =
            std::chrono::duration<double>(samples.back().first - samples.front().first).count();
          if (span >= window) {
            const size_t n = samples.back().second.size();
            double worst_range = 0.0;
            for (size_t i = 0; i < n; ++i) {
              double lo = std::numeric_limits<double>::infinity();
              double hi = -std::numeric_limits<double>::infinity();
              for (const auto& sample : samples) {
                if (sample.second.size() != n) {
                  continue;
                }
                lo = std::min(lo, sample.second[i]);
                hi = std::max(hi, sample.second[i]);
              }
              worst_range = std::max(worst_range, hi - lo);
            }
            measured = samples.back().second;
            max_speed_rad_s = worst_range / span;
            span_sec = span;
            if (max_speed_rad_s <= grasp_arrival_max_speed_rad_s_) {
              return StopWait::STOPPED;
            }
          }
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
    }
    if (stopping()) {
      return StopWait::ABORTED;
    }
    if (samples.empty()) {
      return StopWait::NO_FEEDBACK;
    }
    measured = samples.back().second;
    return StopWait::TIMEOUT;
  }

  // Precise pre-grasp arrival, run after the normal arrival confirmation of the
  // grasp-approach stage. The last streamed command is the planned final pose.
  //   1. wait until every joint has stopped (speed criterion, not a band);
  //   2. residual r = target - measured; inside the tight band -> OK;
  //   3. otherwise re-command command += gain * r (integral style; the lower
  //      controller's steady-state error is a constant offset, so the command
  //      that lands the joint on the target is target + residual), per joint
  //      clamped to target +- grasp_residual_max_rad and to the joint limits,
  //      streamed through the velocity-limited command path; back to 1.
  //   Up to grasp_residual_iterations corrections; a total deadline of
  //   grasp_arrival_timeout_sec. On FAIL the caller does not close the jaws.
  // Logs [GRASP][arrival][PRECISE|COMPENSATE|OK|FAIL|ABORT]. Returns false on
  // FAIL (last_grasp_arrival_failure_ set) and on abort (shutdown/cancel).
  bool preciseGraspArrival(const std::vector<double>& target, const std::string& stage_name) {
    last_grasp_arrival_failure_.clear();
    if (!grasp_precise_arrival_enabled_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRASP][arrival][SKIP] step=%s tzb.grasp_precise_arrival_enabled=false: continuing after the normal arrival band (%.3f rad, J6 %.3f rad); joints may still be moving at the close",
        stage_name.c_str(), arm_motion_goal_tolerance_rad_, jointToleranceRad("J6_joint"));
      return true;
    }
    if (!arm_feedback_enabled_) {
      RCLCPP_WARN(this->get_logger(),
        "[GRASP][arrival][SKIP] step=%s arm feedback disabled: precise arrival cannot be measured",
        stage_name.c_str());
      return true;
    }
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    const size_t n = target.size();
    if (n != group.joint_names.size()) {
      last_grasp_arrival_failure_ = "target_size_mismatch";
      RCLCPP_ERROR(this->get_logger(), "[GRASP][arrival][FAIL] step=%s reason=target_size_mismatch(%zu vs %zu)",
        stage_name.c_str(), n, group.joint_names.size());
      return false;
    }
    const auto lower = robot_model_->lowerBoundsForGroup(arm_group_);
    const auto upper = robot_model_->upperBoundsForGroup(arm_group_);
    const auto t_begin = std::chrono::steady_clock::now();
    const auto deadline = t_begin + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(grasp_arrival_timeout_sec_));
    const auto elapsed = [&t_begin]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_begin).count();
      };
    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][arrival][BEGIN] step=%s tolerance=%.4frad(J6 %.4f) stable=%.2fs max_speed=%.4frad/s timeout=%.1fs gain=%.2f clamp=%.3frad iterations=%d",
      stage_name.c_str(), grasp_arrival_tolerance_rad_, grasp_arrival_tolerance_j6_rad_,
      grasp_arrival_stable_sec_, grasp_arrival_max_speed_rad_s_, grasp_arrival_timeout_sec_,
      grasp_residual_gain_, grasp_residual_max_rad_, grasp_residual_iterations_);

    std::vector<double> command = target;  // last streamed command = planned final
    std::vector<bool> stalled(target.size(), false);
    bool stall_hold_published = false;
    std::vector<double> previous_measured;
    const auto fail = [&](const std::string& reason, const std::vector<double>& measured) {
        last_grasp_arrival_failure_ = reason;
        RCLCPP_ERROR(
          this->get_logger(),
          "[GRASP][arrival][FAIL] step=%s reason=%s elapsed=%.2fs residuals:%s; the gripper is NOT closed",
          stage_name.c_str(), reason.c_str(), elapsed(), formatGraspResiduals(measured, target).c_str());
        return false;
      };

    for (int iter = 0;; ++iter) {
      std::vector<double> measured;
      double max_speed = std::numeric_limits<double>::infinity();
      double span = 0.0;
      const StopWait wait = waitForArmStopped(deadline, measured, max_speed, span);
      if (wait == StopWait::ABORTED) {
        RCLCPP_WARN(this->get_logger(),
          "[GRASP][arrival][ABORT] step=%s iter=%d operation aborted while waiting for the arm to stop; gripper NOT closed",
          stage_name.c_str(), iter);
        return false;
      }
      if (wait == StopWait::NO_FEEDBACK) {
        return fail("no_feedback(no arm state sample within " + std::to_string(grasp_arrival_timeout_sec_) + " s)",
                    previous_measured);
      }
      if (wait == StopWait::TIMEOUT) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "not_stopped(iter=%d max_speed=%.4frad/s > %.4f over %.2fs; timeout %.1fs)",
          iter, max_speed, grasp_arrival_max_speed_rad_s_, span, grasp_arrival_timeout_sec_);
        return fail(buf, measured);
      }

      std::vector<double> residual(n, 0.0);
      double max_err = 0.0;
      size_t worst = 0;
      bool within = true;
      for (size_t i = 0; i < n; ++i) {
        residual[i] = std::remainder(target[i] - measured[i], 2.0 * kPi);
        if (std::abs(residual[i]) > graspArrivalToleranceRad(group.joint_names[i])) {
          within = false;
        }
        if (std::abs(residual[i]) > max_err) {
          max_err = std::abs(residual[i]);
          worst = i;
        }
      }
      double moved = 0.0;
      if (previous_measured.size() == n) {
        for (size_t i = 0; i < n; ++i) {
          moved = std::max(moved, std::abs(std::remainder(measured[i] - previous_measured[i], 2.0 * kPi)));
        }
      }
      RCLCPP_INFO(
        this->get_logger(),
        "[GRASP][arrival][PRECISE] step=%s iter=%d max_err=%.4frad(%s) %s stopped(max_speed=%.4frad/s over %.2fs) moved_since_prev=%.4frad elapsed=%.2fs per-joint:%s",
        stage_name.c_str(), iter, max_err, group.joint_names[worst].c_str(),
        within ? "WITHIN" : "OUTSIDE", max_speed, span, moved, elapsed(),
        formatGraspResiduals(measured, target).c_str());
      if (within) {
        arm_state_ = measured;
        double command_offset = 0.0;
        for (size_t i = 0; i < n; ++i) {
          command_offset = std::max(command_offset, std::abs(command[i] - target[i]));
        }
        RCLCPP_INFO(
          this->get_logger(),
          "[GRASP][arrival][OK] step=%s iter=%d max_err=%.4frad(%s) command_offset_max=%.4frad elapsed=%.2fs: arm stopped inside the precise band; the gripper may close",
          stage_name.c_str(), iter, max_err, group.joint_names[worst].c_str(), command_offset, elapsed());
        return true;
      }
      // Contact stop (real grasp 2026-11-06 #9): during the final descent the jaws can touch the
      // ground/block and stop J2 ~1 cm above the planned TCP height while every joint that sets the
      // jaw position in the horizontal plane and the jaw yaw is on target. Accept that if the arm has
      // stopped, the lateral-plane joints (J1, J4, J6) are within the precise band and the TCP
      // (FK of measured vs target joints) is within grasp_contact_xy_tol_m horizontally and
      // between -grasp_contact_below_m and +grasp_contact_above_m vertically.
      if (grasp_contact_stop_enabled_ && (iter >= grasp_residual_iterations_ || (iter > 0 && moved < kNoMotionNoiseRad) || std::find(stalled.begin(), stalled.end(), true) != stalled.end())) {
        bool lateral_ok = true;
        for (size_t i = 0; i < n; ++i) {
          const auto& jn = group.joint_names[i];
          if ((jn == "J1_joint" || jn == "J4_joint" || jn == "J6_joint") &&
              std::abs(residual[i]) > graspArrivalToleranceRad(jn)) {
            lateral_ok = false;
          }
        }
        try {
          const auto pm = tcpPointForArmJoints(measured);
          const auto pt = tcpPointForArmJoints(target);
          const double dxy = std::hypot(pm.x - pt.x, pm.y - pt.y);
          const double dz = pm.z - pt.z;
          if (lateral_ok && dxy <= grasp_contact_xy_tol_m_ && dz <= grasp_contact_above_m_ && dz >= -grasp_contact_below_m_) {
            arm_state_ = measured;
            // Release the wound-up compensation (it may be pressing a contacting joint into the
            // ground/block): hold the measured pose while the jaws close.
            publishArmCommand(group.joint_names, measured);
            RCLCPP_WARN(this->get_logger(),
              "[GRASP][arrival][OK_CONTACT] step=%s iter=%d tcp_dxy=%.4fm tcp_dz=%+.4fm (limits xy %.3f, z +%.3f/-%.3f) worst_joint=%s err=%.4frad: arm stopped (likely contact); lateral joints on target; the gripper may close",
              stage_name.c_str(), iter, dxy, dz, grasp_contact_xy_tol_m_, grasp_contact_above_m_, grasp_contact_below_m_,
              group.joint_names[worst].c_str(), max_err);
            return true;
          }
          RCLCPP_INFO(this->get_logger(),
            "[GRASP][arrival][CONTACT_CHECK] step=%s iter=%d lateral_ok=%s tcp_dxy=%.4fm tcp_dz=%+.4fm: not accepted",
            stage_name.c_str(), iter, lateral_ok ? "true" : "false", dxy, dz);
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "[GRASP][arrival][CONTACT_CHECK] FK failed: %s", e.what());
        }
      }
      if (iter >= grasp_residual_iterations_) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "residual_not_converged(iter=%d max_err=%.4frad on %s)%s",
          iter, max_err, group.joint_names[worst].c_str(),
          (iter > 0 && moved < kNoMotionNoiseRad) ?
            "; no joint moved for the last correction: lower-controller deadband/min-move" : "");
        return fail(buf, measured);
      }

      // Integral correction of the command, clamped per joint (review 2026-11-06 H2):
      //  - only joints OUTSIDE the precise band are corrected (others keep their command);
      //  - a joint that was corrected last iteration but did not move is STALLED (contact / hard
      //    stop): its command goes back to the measured position and it is never pushed again;
      //  - the command never leads the measured position by more than grasp_residual_lead_max_rad.
      std::vector<double> next = command;
      std::ostringstream oss;
      double max_step = 0.0;
      for (size_t i = 0; i < n; ++i) {
        const bool outside = std::abs(residual[i]) > graspArrivalToleranceRad(group.joint_names[i]);
        const bool was_corrected = previous_measured.size() == n && std::abs(command[i] - target[i]) > 1e-6;
        const bool did_not_move = previous_measured.size() == n &&
          std::abs(std::remainder(measured[i] - previous_measured[i], 2.0 * kPi)) < kNoMotionNoiseRad;
        if (was_corrected && did_not_move) stalled[i] = true;
        if (stalled[i] || !outside) {
          const double hold = stalled[i] ? measured[i] : command[i];
          oss << " " << group.joint_names[i] << (stalled[i] ? "{STALLED hold measured " : "{keep ") << hold << "}";
          max_step = std::max(max_step, stalled[i] ? 0.0 : 0.0);
          next[i] = hold;
          continue;
        }
        const double wanted = command[i] + grasp_residual_gain_ * residual[i];
        double c = std::clamp(wanted, target[i] - grasp_residual_max_rad_, target[i] + grasp_residual_max_rad_);
        c = std::clamp(c, measured[i] - grasp_residual_lead_max_rad_, measured[i] + grasp_residual_lead_max_rad_);
        c = std::clamp(c, lower[static_cast<Eigen::Index>(i)], upper[static_cast<Eigen::Index>(i)]);
        oss << " " << group.joint_names[i] << "{cmd=" << command[i] << "->" << c
            << ", residual=" << residual[i] << (std::abs(c - wanted) > 1e-9 ? ", CLAMPED}" : "}");
        max_step = std::max(max_step, std::abs(c - command[i]));
        next[i] = c;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "[GRASP][arrival][COMPENSATE] step=%s iter=%d max_step=%.4frad:%s",
        stage_name.c_str(), iter + 1, max_step, oss.str().c_str());
      if (max_step < 1e-6) {
        // nothing left to push: hold the measured pose of stalled joints; give the contact check
        // one more settled sample, then fail.
        applyNamedJointPositions(group.joint_names, next);
        publishArmCommand(group.joint_names, next);
        if (!stall_hold_published && std::find(stalled.begin(), stalled.end(), true) != stalled.end()) {
          stall_hold_published = true;
          command = next;
          previous_measured = measured;
          continue;
        }
        return fail("compensation_exhausted(stalled or clamped joints; residual unchanged)", measured);
      }
      trajectory_msgs::msg::JointTrajectory trajectory;
      trajectory.header.stamp = this->now();
      trajectory.joint_names = group.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = next;
      point.time_from_start = durationFromSec(std::max(0.1, max_step / kGraspResidualSpeedRadS));
      trajectory.points.push_back(std::move(point));
      // The stream starts at the last command (the planned final / previous
      // correction), not at the measured pose: re-commanding the measured pose
      // would first move the joint by its own residual.
      applyNamedJointPositions(group.joint_names, command);
      snapshotMeasuredArmStateForMotionCheck();
      if (!publishArmCommandTrajectory(trajectory, stage_name + "_residual" + std::to_string(iter + 1))) {
        RCLCPP_WARN(this->get_logger(),
          "[GRASP][arrival][ABORT] step=%s iter=%d operation aborted during the compensation stream; gripper NOT closed",
          stage_name.c_str(), iter + 1);
        return false;
      }
      command = next;
      previous_measured = measured;
    }
  }

  // The grasp approach: direct pose plan with precise arrival, or (hover > 0)
  // precise arrival at a pose straight above the grasp pose (same orientation),
  // then the descent with precise arrival. A hover pose that cannot be planned
  // or reached falls back to the direct approach; a precise-arrival FAIL at the
  // hover ends the attempt like one at the grasp pose.
  // ---- self-calibration ("hand places, eye sees") mission commands -----------
  // calib_pick: the vision grasp path, then lift to hover and keep holding.
  // calib_place:<yaw_deg> / calib_regrasp:<yaw_deg>: hover above expected_center
  // (z = tzb.grasp_ground_z) with the given tool yaw, descend (precise arrival incl.
  // contact acceptance), open / close, lift to hover; calib_place then parks at observe.
  // calib_observe: observe pose (if not there) + grasp-identical lock, report only.
  // Every result message ends with " json={...}" (FK of MEASURED joints = truth).
  static std::string calibBaseOp(const std::string& command) {
    const auto k = command.find(':');
    return k == std::string::npos ? command : command.substr(0, k);
  }
  static bool isCalibOp(const std::string& op) {
    return op == "calib_pick" || op == "calib_place" || op == "calib_regrasp" || op == "calib_observe" ||
      op == "calib_check" || op == "calib_joints";
  }
  static nlohmann::json mat4Json(const pinocchio::SE3& T) {
    const Eigen::Matrix4d M = T.toHomogeneousMatrix();
    nlohmann::json out = nlohmann::json::array();
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) out.push_back(M(r, c));
    return out;  // row-major 16, same layout as tzb.pose_estimator_reference_transform_matrix
  }
  static bool missionOperationUsesVision(const std::string& op) {
    return op == "grasp" || op == "calib_pick" || op == "calib_observe";
  }
  // Tool yaw convention of the grasp pose: R = Rz(yaw)*Ry(pitch)*Rx(roll), so the
  // x-axis heading is the yaw whatever the forced roll is.
  static double toolYawDeg(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI; }
  static nlohmann::json xyzJson(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }

  nlohmann::json measuredTcpJson() {
    std::vector<double> joints;
    double age = 0.0;
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      if (!have_complete_arm_state_) throw std::runtime_error("no measured arm state for the calibration truth");
      joints = latest_measured_arm_positions_;
      age = std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_arm_state_time_).count();
    }
    const pinocchio::SE3 tcp = framePoseAtArmJoints(tcp_frame_, joints);
    return {{"tcp_xyz", xyzJson(tcp.translation())}, {"tool_yaw_deg", toolYawDeg(tcp.rotation())},
            {"joints", joints}, {"feedback_age_sec", age}};
  }

  nlohmann::json poseJson(const geometry_msgs::msg::Pose& p) {
    return {{"xyz", {p.position.x, p.position.y, p.position.z}},
            {"quat_xyzw", {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}}};
  }

  // The grasp chain evaluated on a locked camera-frame pose, reported in parts.
  nlohmann::json calibTargetJson(const geometry_msgs::msg::PoseStamped& locked) {
    const pinocchio::SE3 reference_raw = graspReferencePoseInPlanningFrame();
    const geometry_msgs::msg::PoseStamped corrected = correctedTargetInPlanningFrame(reference_raw, locked);
    pinocchio::SE3 reference = reference_raw;
    if (apply_reference_transform_to_frozen_frame_) reference = reference * poseEstimatorReferenceTransformSE3();
    const pinocchio::SE3 uncorrected = reference * poseMsgToSE3(preparePoseEstimatorTarget(locked).pose);
    const Eigen::Matrix3d rot = quaternionEigenFromMsg(corrected.pose.orientation).toRotationMatrix();
    const double face_deg = symmetricFaceYaw(rot, corrected.pose.position.x, corrected.pose.position.y) * 180.0 / M_PI;
    nlohmann::json j;
    j["locked_camera"] = poseJson(locked.pose);
    j["locked_camera"]["frame"] = locked.header.frame_id;
    j["locked_camera"]["stamp"] = rclcpp::Time(locked.header.stamp).seconds();
    j["corrected"] = poseJson(corrected.pose);
    j["corrected"]["face_yaw_deg"] = face_deg;
    j["corrected"]["final_yaw_deg"] = face_deg + grasp_yaw_offset_deg_;
    j["corrected"]["zyx_yaw_deg"] = toolYawDeg(rot);
    j["uncorrected"] = {{"xyz", xyzJson(uncorrected.translation())}, {"zyx_yaw_deg", toolYawDeg(uncorrected.rotation())}};
    j["camera_link_in_base"] = poseJson(poseMsgFromSE3(reference_raw));
    j["grasp_yaw_offset_deg"] = grasp_yaw_offset_deg_;
    j["grasp_ground_z"] = grasp_ground_z_;
    return j;
  }

  geometry_msgs::msg::PoseStamped calibToolPose(double x, double y, double z, double yaw_deg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = planning_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = quaternionMsgFromRpy(
      force_target_roll_ ? forced_target_roll_ : 0.0,
      force_target_pitch_ ? forced_target_pitch_ : 0.0,
      yaw_deg * M_PI / 180.0);
    return pose;
  }

  bool measuredNearJoints(const std::vector<double>& ref) {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    return have_complete_arm_state_ && latest_measured_arm_positions_.size() == ref.size() &&
      maxNormalizedJointError(latest_measured_arm_positions_, ref) <= 1.0;
  }

  bool runCalibPick() {
    calib_hold_after_close_ = true;
    bool ok = false;
    try {
      ok = runSequenceOnce();
    } catch (...) {
      calib_hold_after_close_ = false;
      throw;
    }
    calib_hold_after_close_ = false;
    return ok;
  }

  // Called by runSequenceOnce right after the gripper closed (calib_pick only).
  bool calibFinishPick() {
    nlohmann::json j;
    j["op"] = "calib_pick";
    const auto& p = e3_target_world_pose_.pose;
    j["commanded"] = poseJson(p);
    j["commanded"]["yaw_deg"] = toolYawDeg(quaternionEigenFromMsg(p.orientation).toRotationMatrix());
    try {
      j["measured_close"] = measuredTcpJson();
      j.update(calibTargetJson(calib_locked_pose_));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[CALIB][pick] truth/chain report failed: %s", e.what());
      mission_result_json_ = j.dump();
      return false;
    }
    auto hover = e3_target_world_pose_;
    hover.pose.position.z += grasp_hover_height_m_;
    calib_current_hover_ = hover;
    if (!executeArmPosePlan(hover, false, "calib_pick_lift_to_hover", true)) {
      mission_result_json_ = j.dump();
      return false;
    }
    try { j["hover_measured"] = measuredTcpJson(); } catch (const std::exception&) {}
    mission_result_json_ = j.dump();
    RCLCPP_INFO(this->get_logger(), "[CALIB][pick][OK] holding at hover: %s", mission_result_json_.c_str());
    return true;
  }

  bool runCalibPlaceOrRegrasp(bool close_instead_of_open) {
    const std::string tag = close_instead_of_open ? "calib_regrasp" : "calib_place";
    std::vector<double> center;
    double yaw_deg = 0.0;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      center = mission_expected_center_;
      yaw_deg = mission_calib_yaw_deg_;
    }
    if (center.size() != 3 || !std::isfinite(grasp_ground_z_)) {
      RCLCPP_ERROR(this->get_logger(), "[CALIB][%s] needs expected_center and a finite tzb.grasp_ground_z", tag.c_str());
      return false;
    }
    if (close_instead_of_open == gripper_closed_on_object_.load()) {
      RCLCPP_ERROR(this->get_logger(), "[CALIB][%s] gripper state mismatch: object_held=%s", tag.c_str(),
        gripper_closed_on_object_.load() ? "true" : "false");
      return false;
    }
    replay_segments_.clear();
    last_grasp_arrival_failure_.clear();
    const auto target = calibToolPose(center[0], center[1], grasp_ground_z_, yaw_deg);
    nlohmann::json j;
    j["op"] = tag;
    j["commanded"] = poseJson(target.pose);
    j["commanded"]["yaw_deg"] = yaw_deg;
    mission_result_json_ = j.dump();
    setPhase("prepare");
    if (!readInitialArmJointPositions(arm_state_)) return false;
    try { resetTcpTrajectories(arm_state_); } catch (const std::exception&) { record_tcp_trajectory_.store(false); }
    if (e3_collision_object_enabled_) {
      removeObject(e3_object_id_, false);  // the block is in the jaws (place) or will be re-added below
      if (close_instead_of_open) {
        if (!addMeshObject(e3_object_id_, e3_mesh_resource_, target)) return false;
        if (!setGripperE3CollisionAllowance(grasp_collision_allowed_penetration_m_,
              "ignore_e3_collision_with_both_gripper_links")) return false;
      }
    }
    // User 2026-11-06 22:55: with the block in the jaws the arm must NOT go through the observe pose
    // (the fold hits the lidar and knocks the block crooked). calib_place (block held, arm at the
    // previous hover) goes hover -> hover directly; only the empty-gripper calib_regrasp starts from observe.
    if (close_instead_of_open && !measuredNearJoints(observe_joint_positions_)) {
      setPhase("observe");
      if (!executeArmJointPlan(observe_joint_positions_, tag + "_move_arm_to_observe")) return false;
    } else if (!close_instead_of_open) {
      RCLCPP_INFO(this->get_logger(), "[CALIB][%s] block held: direct move to the new hover (no observe pose)", tag.c_str());
    }
    setPhase("grasp");
    auto hover = target;
    hover.pose.position.z += grasp_hover_height_m_;
    calib_current_hover_ = hover;
    if (!executeArmPosePlan(hover, false, tag + "_move_tcp_to_hover", true)) return false;
    if (!executeArmPosePlan(target, false, tag + "_move_tcp_to_target", true)) {
      if (!last_grasp_arrival_failure_.empty())
        RCLCPP_ERROR(this->get_logger(), "[CALIB][%s] precise arrival failed: %s", tag.c_str(), last_grasp_arrival_failure_.c_str());
      return false;
    }
    try {
      if (close_instead_of_open) {
        j["measured_close"] = measuredTcpJson();
        publishGripperCommand(tag + "_close_gripper", 1);
        if (gripper_settle_sec_ > 0.0) sleepAfterAction(gripper_settle_sec_);
        j["measured_close_after_settle"] = measuredTcpJson();
      } else {
        j["measured_open"] = measuredTcpJson();
        publishGripperCommand(tag + "_open_gripper", -1);
        sleepAfterAction(1.0);
        j["measured_open_after_1s"] = measuredTcpJson();
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[CALIB][%s] truth report failed: %s", tag.c_str(), e.what());
      mission_result_json_ = j.dump();
      return false;
    }
    mission_result_json_ = j.dump();
    if (!executeArmPosePlan(hover, false, tag + "_lift_to_hover", true)) return false;
    try { j["hover_measured"] = measuredTcpJson(); } catch (const std::exception&) {}
    mission_result_json_ = j.dump();
    if (!close_instead_of_open) {
      if (e3_collision_object_enabled_) removeObject(e3_object_id_, false);
      setPhase("observe");
      if (!executeArmJointPlan(observe_joint_positions_, "move_arm_to_observe_before_perception")) return false;
    }
    RCLCPP_INFO(this->get_logger(), "[CALIB][%s][OK] %s", tag.c_str(), mission_result_json_.c_str());
    return true;
  }

  // calib_check:<yaw_deg>: plan-only reachability of hover + target above expected_center
  // from the (planned) observe joints. No motion. json: reachable, hover_joints, target_joints.
  bool planOnlyPose(const geometry_msgs::msg::PoseStamped& target, const std::vector<double>& start,
                    std::vector<double>& final_joints, std::string& err) {
    auto req = std::make_shared<lite_motion_msgs::srv::PlanArmMotion::Request>();
    req->group_name = arm_group_;
    req->ee_frame = tcp_frame_;
    req->use_pose_goal = true;
    req->target_pose = target;
    req->start_joint_positions = start;
    req->plan_only = true;
    req->use_cartesian_path = false;
    req->cartesian_max_step = 0.0;
    req->use_weighted_orientation_goal = position_only_target_;
    req->orientation_weight_x = 0.0;
    req->orientation_weight_y = 0.0;
    req->orientation_weight_z = 0.0;
    req->use_request_solver_settings = true;
    req->request_planning_timeout = joint_planning_timeout_;
    req->request_path_check_step = path_check_step_m_;
    auto resp = callService<lite_motion_msgs::srv::PlanArmMotion>(plan_client_, req, "plan_arm_motion");
    if (!resp || !resp->success || resp->trajectory.points.empty()) {
      err = !resp ? "no response" : (!resp->success ? resp->message : "empty trajectory");
      return false;
    }
    final_joints = resp->trajectory.points.back().positions;
    return true;
  }

  bool runCalibCheck() {
    std::vector<double> center;
    double yaw_deg = 0.0;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      center = mission_expected_center_;
      yaw_deg = mission_calib_yaw_deg_;
    }
    nlohmann::json j;
    j["op"] = "calib_check";
    j["reachable"] = false;
    if (center.size() != 3 || !std::isfinite(grasp_ground_z_)) {
      j["error"] = "needs expected_center and a finite tzb.grasp_ground_z";
      mission_result_json_ = j.dump();
      return false;
    }
    const auto target = calibToolPose(center[0], center[1], grasp_ground_z_, yaw_deg);
    auto hover = target;
    hover.pose.position.z += grasp_hover_height_m_;
    j["commanded"] = poseJson(target.pose);
    j["commanded"]["yaw_deg"] = yaw_deg;
    std::vector<double> hover_joints, target_joints;
    std::string err;
    if (!planOnlyPose(hover, observe_joint_positions_, hover_joints, err)) {
      j["error"] = "hover: " + err;
    } else if (!planOnlyPose(target, hover_joints, target_joints, err)) {
      j["hover_joints"] = hover_joints;
      j["error"] = "target: " + err;
    } else {
      j["reachable"] = true;
      j["hover_joints"] = hover_joints;
      j["target_joints"] = target_joints;
    }
    mission_result_json_ = j.dump();
    RCLCPP_INFO(this->get_logger(), "[CALIB][check] %s", mission_result_json_.c_str());
    return true;  // the check itself succeeded; reachability is in the json
  }

  bool runCalibObserve() {
    double observe_j1 = 0.0;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      observe_j1 = mission_calib_observe_j1_;
    }
    nlohmann::json j;
    j["op"] = "calib_observe";
    j["locked"] = false;
    j["observe_j1"] = observe_j1;
    j["camera_frame_from_measured_joints"] = camera_frame_from_measured_joints_;
    mission_result_json_ = j.dump();
    setPhase("prepare");
    if (!readInitialArmJointPositions(arm_state_)) return false;
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      resetPoseEstimatorCaptureLocked();
      have_frozen_pose_estimator_reference_ = false;
      lock_measured_joints_valid_ = false;
    }
    std::vector<double> observe_joints = observe_joint_positions_;
    if (!observe_joints.empty()) observe_joints[0] = observe_j1;
    j["observe_joints_commanded"] = observe_joints;
    setPhase("observe");
    // Review H4: reach observe exactly like the production grasp (via ini) so the lower controller's
    // direction-dependent steady-state error at observe is the same as in production.
    if (!executeArmJointPlan(initial_arm_positions_, "move_arm_to_ini_joint_positions") ||
        !executeArmJointPlan(observe_joints, "move_arm_to_observe_before_perception")) {
      return false;
    }
    setPhase("locate");
    if (!freezePoseEstimatorReferenceAtCurrentArmState()) return false;
    if (!enablePoseEstimatorCaptureAtCurrentReferencePose()) return false;
    try { j["arm_measured"] = measuredTcpJson(); } catch (const std::exception&) {}
    j["reference_joints"] = observed_reference_joints_;
    try {
      j["camera_link_fk_planned"] = mat4Json(framePoseAtArmJoints(pose_estimator_reference_frame_, observed_reference_joints_));
    } catch (const std::exception&) {}
    geometry_msgs::msg::PoseStamped locked;
    const bool got = waitForPoseEstimatorTarget(locked);
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (lock_measured_joints_valid_) {
        j["lock_measured_joints"] = lock_measured_joints_;
        j["camera_link_fk_measured"] = mat4Json(lock_measured_camera_in_planning_);
        try {
          const pinocchio::SE3 tcp = framePoseAtArmJoints(tcp_frame_, lock_measured_joints_);
          j["tcp_fk_measured"] = {{"tcp_xyz", xyzJson(tcp.translation())}, {"tool_yaw_deg", toolYawDeg(tcp.rotation())}};
        } catch (const std::exception&) {}
      }
      resetPoseEstimatorCaptureLocked();
    }
    if (stopping()) return false;
    if (got) {
      j["locked"] = true;
      try {
        j.update(calibTargetJson(locked));
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[CALIB][observe] chain report failed: %s", e.what());
        j["error"] = e.what();
      }
    }
    mission_result_json_ = j.dump();
    RCLCPP_INFO(this->get_logger(), "[CALIB][observe][%s] %s", got ? "LOCKED" : "NO_LOCK", mission_result_json_.c_str());
    return true;
  }

  // calib_joints:<j1>,...,<j6> (rad): validates the command and fills `joints`; returns "" when it
  // may run, else the rejection reason. executeArmJointPlan CLAMPS goals to the URDF limits
  // silently, so every limit is checked here, strictly.
  static constexpr double kCalibJointsMinJ2 = -1.05;  // real J2 hard stop ~ -1.06, URDF says -1.1345
  static constexpr double kCalibJointsMaxAbsJ1 = 1.0;
  std::string parseCalibJointsCommand(const std::string& command, std::vector<double>& joints) {
    const auto k = command.find(':');
    if (k == std::string::npos) return "needs the form calib_joints:<j1>,<j2>,<j3>,<j4>,<j5>,<j6> (rad)";
    joints.clear();
    std::stringstream ss(command.substr(k + 1));
    std::string token;
    while (std::getline(ss, token, ',')) {
      size_t used = 0;
      double v = 0.0;
      try { v = std::stod(token, &used); } catch (const std::exception&) { used = 0; }
      if (used == 0 || used != token.size()) return "joint value '" + token + "' is not a number";
      if (!std::isfinite(v)) return "joint value '" + token + "' is not finite";
      joints.push_back(v);
    }
    if (!command.empty() && command.back() == ',') return "trailing ',' (needs exactly 6 joint values)";
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    if (joints.size() != 6 || group.joint_names.size() != 6)
      return "needs exactly 6 joint values, got " + std::to_string(joints.size());
    const auto lower = robot_model_->lowerBoundsForGroup(arm_group_);
    const auto upper = robot_model_->upperBoundsForGroup(arm_group_);
    for (size_t i = 0; i < joints.size(); ++i) {
      if (joints[i] < lower[static_cast<Eigen::Index>(i)] || joints[i] > upper[static_cast<Eigen::Index>(i)])
        return "J" + std::to_string(i + 1) + "=" + std::to_string(joints[i]) + " outside planner limits [" +
          std::to_string(lower[static_cast<Eigen::Index>(i)]) + ", " + std::to_string(upper[static_cast<Eigen::Index>(i)]) + "]";
    }
    if (joints[1] < kCalibJointsMinJ2)
      return "J2=" + std::to_string(joints[1]) + " below the safety limit " + std::to_string(kCalibJointsMinJ2) +
        " (real J2 hard stop ~ -1.06)";
    if (std::abs(joints[0]) > kCalibJointsMaxAbsJ1)
      return "|J1|=" + std::to_string(std::abs(joints[0])) + " exceeds the safety limit " + std::to_string(kCalibJointsMaxAbsJ1);
    if (gripper_closed_on_object_.load())
      return "gripper is holding an object; moving it through arbitrary poses can hit the lidar";
    return "";
  }

  // Per-joint median of the measured arm joints over `seconds` (distinct feedback samples only).
  bool collectSettledJoints(double seconds, std::vector<double>& median) {
    std::vector<std::vector<double>> samples;
    uint64_t last_sequence = 0;
    bool first = true;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!stopping() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(arm_state_mutex_);
        if (have_complete_arm_state_ && (first || arm_state_sequence_ != last_sequence)) {
          first = false;
          last_sequence = arm_state_sequence_;
          samples.push_back(latest_measured_arm_positions_);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (stopping() || samples.empty()) return false;
    median.assign(samples.front().size(), 0.0);
    for (size_t i = 0; i < median.size(); ++i) {
      std::vector<double> column;
      for (const auto& s : samples) column.push_back(s[i]);
      std::sort(column.begin(), column.end());
      const size_t n = column.size();
      median[i] = n % 2 ? column[n / 2] : 0.5 * (column[n / 2 - 1] + column[n / 2]);
    }
    return true;
  }

  // calib_joints:<j1..j6>: pure joint-space move (planned, collision-checked) + settle + report.
  // No perception, no target lock. success = arrived and max |measured - target| <= 0.03 rad.
  bool runCalibJoints() {
    constexpr double kMaxAbsErrRad = 0.10;  // real J1 stops ~0.03 short (deadband); the capture uses the MEASURED joints
    constexpr double kSettleSec = 0.6;
    std::vector<double> target;
    {
      std::lock_guard<std::mutex> lock(mission_mutex_);
      target = mission_calib_joints_;
    }
    nlohmann::json j;
    j["op"] = "calib_joints";
    j["target_joints"] = target;
    j["planning_frame"] = planning_frame_;
    j["camera_frame"] = pose_estimator_reference_frame_;
    j["success"] = false;
    j["reason"] = "move not completed";
    mission_result_json_ = j.dump();
    setPhase("prepare");
    if (!readInitialArmJointPositions(arm_state_)) {
      j["reason"] = "no measured arm state";
      mission_result_json_ = j.dump();
      return false;
    }
    try {
      j["camera_link_fk_target"] = mat4Json(framePoseAtArmJoints(pose_estimator_reference_frame_, target));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[CALIB][joints] target FK failed: %s", e.what());
    }
    setPhase("observe");
    last_plan_failure_.clear();
    bool moved = executeArmJointPlan(target, "move_arm_to_calib_joints");
    if (!moved && !stopping() && !last_plan_failure_.empty()) {
      RCLCPP_WARN(this->get_logger(), "[CALIB][joints] direct plan failed (%s); retrying once via pre_ini",
        last_plan_failure_.c_str());
      moved = executeArmJointPlan(initial_arm_positions_, "move_arm_to_ini_joint_positions") &&
        executeArmJointPlan(target, "move_arm_to_calib_joints");
    }
    if (stopping()) return false;
    std::string reason;
    if (!moved) {
      reason = last_plan_failure_.empty() ? "arm did not arrive at the target (see tzb_catch log)"
                                          : "plan_failed(" + last_plan_failure_ + ")";
    }
    std::vector<double> settled;
    if (!collectSettledJoints(kSettleSec, settled)) {
      if (stopping()) return false;
      j["reason"] = reason.empty() ? "no arm feedback while settling" : reason;
      mission_result_json_ = j.dump();
      return false;
    }
    double max_err = 0.0;
    nlohmann::json err_per_joint = nlohmann::json::array();
    for (size_t i = 0; i < settled.size() && i < target.size(); ++i) {
      err_per_joint.push_back(std::abs(settled[i] - target[i]));
      max_err = std::max(max_err, std::abs(settled[i] - target[i]));
    }
    j["settled_joints_median"] = settled;
    j["abs_err_rad"] = err_per_joint;
    j["max_abs_err_rad"] = max_err;
    try { j["arm_measured"] = measuredTcpJson(); } catch (const std::exception&) {}
    try {
      j["camera_link_fk_measured"] = mat4Json(framePoseAtArmJoints(pose_estimator_reference_frame_, settled));
      j["tcp_fk_measured"] = mat4Json(framePoseAtArmJoints(tcp_frame_, settled));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[CALIB][joints] measured FK failed: %s", e.what());
    }
    if (reason.empty() && max_err > kMaxAbsErrRad) {
      reason = "settled joints " + std::to_string(max_err) + " rad away from the target (limit " +
        std::to_string(kMaxAbsErrRad) + ")";
    }
    const bool ok = reason.empty();
    j["success"] = ok;
    j["reason"] = ok ? "" : reason;
    mission_result_json_ = j.dump();
    RCLCPP_INFO(this->get_logger(), "[CALIB][joints][%s] %s", ok ? "OK" : "FAIL", mission_result_json_.c_str());
    return ok;
  }

  bool approachGraspPose(const geometry_msgs::msg::PoseStamped& grasp_pose) {
    if (grasp_hover_height_m_ > 0.0) {
      auto hover = grasp_pose;
      hover.pose.position.z += grasp_hover_height_m_;
      RCLCPP_INFO(this->get_logger(),
        "[GRASP][hover][BEGIN] height=%.3fm hover_xyz=(%.4f, %.4f, %.4f) grasp_xyz=(%.4f, %.4f, %.4f)",
        grasp_hover_height_m_, hover.pose.position.x, hover.pose.position.y, hover.pose.position.z,
        grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
      if (executeArmPosePlan(hover, false, "move_tcp_to_hover", true)) {
        RCLCPP_INFO(this->get_logger(), "[GRASP][hover][OK] descending to the grasp pose");
        return executeArmPosePlan(grasp_pose, false, "move_tcp_to_target", true);
      }
      if (stopping() || !last_grasp_arrival_failure_.empty()) {
        return false;
      }
      RCLCPP_WARN(this->get_logger(),
        "[GRASP][hover][FALLBACK] hover pose not reached (%s); direct approach to the grasp pose",
        last_plan_failure_.empty() ? lastStageDiagnosis().c_str() : last_plan_failure_.c_str());
    }
    return executeArmPosePlan(grasp_pose, false, "move_tcp_to_target", true);
  }

  bool runSequenceOnce() {
    applyTargetClassProfile();
    RCLCPP_INFO(this->get_logger(), "[GRASP][task][BEGIN] target_source=%s arm_command=%s",
      use_yaml_e3_target_ ? "yaml" : pose_estimator_target_topic_.c_str(),
      arm_command_enabled_ ? "enabled" : "disabled");
    replay_segments_.clear();
    last_grasp_arrival_failure_.clear();
    if (e3_collision_object_enabled_) {
      removeObject(e3_object_id_, false);
    }
    e3_attached_to_tcp_ = false;
    e3_target_world_pose_ = geometry_msgs::msg::PoseStamped();
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      resetPoseEstimatorCaptureLocked();
      have_frozen_pose_estimator_reference_ = false;
    }
    observed_reference_joints_.clear();

    setPhase("prepare");
    if (!readInitialArmJointPositions(arm_state_)) {
      return false;
    }
    try {
      resetTcpTrajectories(arm_state_);
    } catch (const std::exception& e) {
      record_tcp_trajectory_.store(false);
      RCLCPP_ERROR(this->get_logger(), "Unable to initialize TCP trajectory recording: %s", e.what());
      return false;
    }
    publishCurrentTheoreticalJointState("physical_initial_arm_state");
    publishReferenceMarkers("physical_initial_arm_state");

    if (!executeArmJointPlan(initial_arm_positions_, "move_arm_to_ini_joint_positions")) {
      return false;
    }

    setPhase("observe");
    if (!executeArmJointPlan(
          observe_joint_positions_, "move_arm_to_observe_before_perception")) {
      return false;
    }
    publishGripperCommand("open_gripper_at_observe_joint_positions", -1);

    setPhase("locate");
    if (!use_yaml_e3_target_) {
      if (!freezePoseEstimatorReferenceAtCurrentArmState()) {
        return false;
      }

      // A pose can be geometrically valid while still being rejected by the
      // arm planner.  Keep acquiring fresh estimator poses until one can be
      // planned and physically reached.  The continuity timer is restarted
      // for every retry, so a candidate is only accepted after the estimator
      // has been publishing it continuously for the configured duration.
      bool grasp_target_reached = false;
      int target_attempt = 0;
      while (!stopping()) {
        if (max_target_attempts_ > 0 && target_attempt >= max_target_attempts_) {
          RCLCPP_ERROR(
            this->get_logger(),
            "[GRASP][task][ABORT] target acquisition exhausted %d attempts; giving up this task.",
            max_target_attempts_);
          break;
        }
        ++target_attempt;
        if (!enablePoseEstimatorCaptureAtCurrentReferencePose()) {
          return false;
        }

        geometry_msgs::msg::PoseStamped locked_pose;
        if (!waitForPoseEstimatorTarget(locked_pose)) {
          if (stopping()) {
            return false;
          }
          RCLCPP_WARN(
            this->get_logger(),
            "Pose estimator did not provide a continuous target within %.2f s; retrying pose acquisition.",
            pose_estimator_wait_timeout_sec_);
          continue;
        }

        calib_locked_pose_ = locked_pose;
        if (!resolveE3TargetPose(e3_target_world_pose_)) {
          RCLCPP_WARN(
            this->get_logger(),
            "Could not transform the pose estimator target; retrying pose acquisition.");
          continue;
        }
        e3_target_world_pose_.header.frame_id = planning_frame_;
        publishReferenceMarkers("resolved_tcp_target");
        publishE3Marker();

        if (e3_collision_object_enabled_) {
          // Remove any object left by a previous rejected candidate before
          // installing the next candidate in the planning scene.
          removeObject(e3_object_id_, false);
          if (!addMeshObject(e3_object_id_, e3_mesh_resource_, e3_target_world_pose_)) {
            return false;
          }
          if (!setGripperE3CollisionAllowance(
                grasp_collision_allowed_penetration_m_,
                "ignore_e3_collision_with_both_gripper_links")) {
            return false;
          }
        }

        setPhase("grasp");
        if (approachGraspPose(e3_target_world_pose_)) {
          grasp_target_reached = true;
          break;
        }
        if (!last_grasp_arrival_failure_.empty()) {
          // The arm is at the coarse grasp pose with the jaws open: no close and
          // no re-acquisition from here; the attempt ends (mission: recovery to
          // pre_ini with arrival confirmation).
          RCLCPP_ERROR(
            this->get_logger(),
            "[GRASP][task][ABORT] step=move_tcp_to_target precise arrival failed: %s; gripper not closed, aborting this grasp attempt",
            last_grasp_arrival_failure_.c_str());
          return false;
        }
        setPhase("locate");

        RCLCPP_WARN(
          this->get_logger(),
          "[GRASP][task][RETRY] step=move_tcp_to_target failed; see preceding plan/command/feedback error; next=target_acquisition (existing retry loop has no attempt limit).");
        if (e3_collision_object_enabled_) {
          removeObject(e3_object_id_, false);
        }
        {
          std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
          resetPoseEstimatorCaptureLocked();
        }
      }

      if (!grasp_target_reached) {
        return false;
      }
    } else {
      if (!resolveE3TargetPose(e3_target_world_pose_)) {
        return false;
      }
      e3_target_world_pose_.header.frame_id = planning_frame_;
      publishReferenceMarkers("resolved_tcp_target");
      publishE3Marker();

      if (e3_collision_object_enabled_) {
        if (!addMeshObject(e3_object_id_, e3_mesh_resource_, e3_target_world_pose_)) {
          return false;
        }
        if (!setGripperE3CollisionAllowance(
              grasp_collision_allowed_penetration_m_,
              "ignore_e3_collision_with_both_gripper_links")) {
          return false;
        }
      }

      setPhase("grasp");
      if (!approachGraspPose(e3_target_world_pose_)) {
        if (!last_grasp_arrival_failure_.empty()) {
          RCLCPP_ERROR(
            this->get_logger(),
            "[GRASP][task][ABORT] step=move_tcp_to_target precise arrival failed: %s; gripper not closed, aborting this grasp attempt",
            last_grasp_arrival_failure_.c_str());
        }
        return false;
      }
    }
    publishGripperCommand(
      "close_gripper_at_target",
      static_cast<int8_t>(gripper_close_level_));
    if (gripper_settle_sec_ > 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "[GRASP][gripper][SETTLE] waiting %.2fs for the jaws to close before moving the arm.",
        gripper_settle_sec_);
      sleepAfterAction(gripper_settle_sec_);
    }
    if (calib_hold_after_close_) {
      return calibFinishPick();
    }
    if (!completePlaceAndReturn("")) {
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "tzb_catch sequence completed: target reached, gripper closed, arm moved to tzb.place_joint_positions, gripper opened, and arm returned to tzb.pre_ini_joint_positions.");
    return true;
  }

  // The completion steps once the object is in the jaws: staged move to the
  // place pose (with the existing retry through ini), release, staged return to
  // pre_ini, every stage with physical confirmation. Shared by the normal
  // sequence, the shutdown sequence and the action cancel path (H3): an object
  // is never carried to pre_ini.
  bool completePlaceAndReturn(const std::string& stage_prefix) {
    setPhase("place");
    if (!executeArmStagedYawPlan(place_arm_positions_, stage_prefix + "move_arm_to_place_joint_positions")) {
      if (stopping()) {
        return false;  // aborted: no retry motion
      }
      if (restricted_recovery_.load()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[GRASP][retract][NO_FALLBACK] step=%smove_arm_to_place_joint_positions failed; the ini-recover/retry path is disabled during shutdown/cancel recovery (object still in the gripper).",
          stage_prefix.c_str());
        return false;
      }
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to reach place_joint_positions; returning to ini_joint_positions before retrying place.");
      if (!executeArmJointPlan(
            initial_arm_positions_, stage_prefix + "recover_to_ini_before_place_retry")) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Could not recover to ini_joint_positions after place planning failed; keeping gripper closed.");
        return false;
      }
      if (!executeArmStagedYawPlan(
            place_arm_positions_, stage_prefix + "retry_move_arm_to_place_joint_positions")) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Place retry failed after recovery to ini_joint_positions; keeping gripper closed.");
        return false;
      }
    }
    publishGripperCommand(stage_prefix + "open_gripper_at_place_joint_positions", -1);

    setPhase("home");
    if (!executeArmStagedYawPlan(
          pre_initial_arm_positions_, stage_prefix + "move_arm_to_pre_ini_after_place")) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to return to pre_ini_joint_positions after opening the gripper.");
      return false;
    }
    return true;
  }

  // Why the last recovery/homing stage failed, from the stage diagnostics.
  std::string recoveryFailureReason() const {
    if (last_service_timeout_) return "planner_unavailable(no response within " + std::to_string(static_cast<int>(kRestrictedResponseWaitSec)) + " s)";
    const std::string diagnosis = lastStageDiagnosis();
    if (!diagnosis.empty()) return diagnosis;
    if (!last_plan_failure_.empty()) return "plan_failed(" + last_plan_failure_ + ")";
    return "execute_failed";
  }

  // Feedback classification of the last arrival wait (reset by every wait);
  // "" when the last stage was reached or failed for another reason.
  std::string lastStageDiagnosis() const {
    if (last_no_motion_) return kNoMotionReason;
    if (!last_stuck_joints_.empty()) return "stuck_joint(" + last_stuck_joints_ + ")";
    if (!last_deadband_joints_.empty()) return "deadband_or_offset(" + last_deadband_joints_ + ")";
    if (last_feedback_stale_) return "feedback_stale";
    return "";
  }

  // Common pre-checks of the unattended recovery paths (shutdown homing and
  // action cancel). Returns "" when a motion may be attempted.
  std::string recoveryPrecheck() {
    if (!arm_command_sent_.load()) return "never_commanded";
    if (!plan_client_->wait_for_service(std::chrono::seconds(1))) return "planner_unavailable";
    if (!readInitialArmJointPositions(arm_state_)) return "no_arm_feedback";
    return "";
  }

  bool waitForTaskStart() {
    if (!task_start_gate_enabled_) {
      RCLCPP_INFO(this->get_logger(), "[GRASP][trigger][ACCEPTED] start_gate=disabled");
      return true;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[GRASP][trigger][WAIT] navigation_topic=%s or start_topic=%s value=1 (check navigation listener setting).",
      navigation_goal_status_topic_.c_str(),
      task_start_topic_.c_str());
    setPhase("waiting");
    std::unique_lock<std::mutex> lock(task_trigger_mutex_);
    task_trigger_cv_.wait(lock, [this]() {
      return stopping() || task_trigger_pending_;
    });
    if (stopping()) {
      return false;
    }
    task_trigger_pending_ = false;
    RCLCPP_INFO(this->get_logger(), "[GRASP][trigger][ACCEPTED] pending_trigger consumed");
    return true;
  }

  void onArm2NavState(const arm2nav_msgs::msg::Arm2Nav& msg) {
    if (!sentry_mode_) {
      return;
    }
    bool should_notify = false;
    {
      std::lock_guard<std::mutex> lock(sentry_mutex_);
      latest_arm2nav_state_ = msg;
      if (msg.arm_trigger && !last_arm2nav_arm_trigger_ && !sentry_task_active_) {
        sentry_trigger_pending_ = true;
        should_notify = true;
      }
      last_arm2nav_arm_trigger_ = msg.arm_trigger;
    }
    if (should_notify) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received arm_trigger=1 from arm2nav; starting the sentry arm task.");
      sentry_cv_.notify_all();
    }
  }

  void publishArm2NavState(
    bool target_need_stop,
    bool nav_trigger,
    bool return_home,
    bool arm_trigger = false) {
    if (!arm2nav_arm_state_pub_) {
      return;
    }
    arm2nav_msgs::msg::Arm2Nav latest_state;
    {
      std::lock_guard<std::mutex> lock(sentry_mutex_);
      latest_state = latest_arm2nav_state_;
    }
    arm2nav_msgs::msg::Arm2Nav msg;
    msg.target_need_stop = target_need_stop;
    msg.suggest_x = latest_state.suggest_x;
    msg.suggest_y = latest_state.suggest_y;
    msg.suggest_yaw = latest_state.suggest_yaw;
    msg.nav_trigger = nav_trigger;
    msg.arm_trigger = arm_trigger;
    msg.return_home = return_home;
    arm2nav_arm_state_pub_->publish(msg);
  }

  bool runSentrySequence() {
    if (!readInitialArmJointPositions(arm_state_)) {
      return false;
    }
    if (!executeArmJointPlan(sentry_joint_positions_, "move_arm_to_sentry_joint_positions")) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to reach sentry_joint_positions; sentry mode cannot start.");
      return false;
    }
    publishGripperCommand("open_gripper_at_sentry_joint_positions", -1);
    publishArm2NavState(false, false, false);
    RCLCPP_INFO(
      this->get_logger(),
      "Sentry mode ready at sentry_joint_positions; waiting for arm2nav arm_trigger.");

    int completed_count = 0;
    while (!stopping()) {
      {
        std::unique_lock<std::mutex> lock(sentry_mutex_);
        sentry_cv_.wait(lock, [this]() {
          return stopping() || sentry_trigger_pending_;
        });
        if (stopping()) {
          return false;
        }
        sentry_trigger_pending_ = false;
        sentry_task_active_ = true;
      }

      // Acknowledge the navigation command and clear target_need_stop while
      // the arm task is running. The arm2nav bridge clears its command latch
      // when it receives this arm-side state.
      publishArm2NavState(false, false, false);
      const bool success = runSequenceOnce();
      {
        std::lock_guard<std::mutex> lock(sentry_mutex_);
        sentry_task_active_ = false;
      }

      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Sentry arm task failed; returning to sentry_joint_positions and waiting for the next arm_trigger.");
        if (!executeArmJointPlan(
              sentry_joint_positions_, "recover_to_sentry_joint_positions")) {
          return false;
        }
        continue;
      }

      ++completed_count;
      const bool return_home = completed_count >= sentry_execution_limit_;
      RCLCPP_INFO(
        this->get_logger(),
        "Sentry arm task completed: %d/%d.",
        completed_count, sentry_execution_limit_);
      publishArm2NavState(false, true, return_home);
      std::this_thread::sleep_for(
        std::chrono::duration<double>(sentry_trigger_pulse_sec_));
      publishArm2NavState(false, false, return_home);

      if (return_home) {
        RCLCPP_INFO(
          this->get_logger(),
          "Sentry execution limit reached; published return_home=1.");
        return true;
      }
      if (!executeArmJointPlan(
            sentry_joint_positions_, "return_to_sentry_joint_positions")) {
        return false;
      }
    }
    return false;
  }

  void publishTaskResult(bool success) {
    if (success) {
      RCLCPP_INFO(this->get_logger(), "[GRASP][task][END] result=SUCCESS result_publisher=%s",
        task_result_pub_ ? "enabled" : "disabled");
    } else {
      const std::string diagnosis = lastStageDiagnosis();
      RCLCPP_ERROR(this->get_logger(), "[GRASP][task][END] result=FAIL%s%s result_publisher=%s",
        diagnosis.empty() ? "" : " reason=", diagnosis.c_str(), task_result_pub_ ? "enabled" : "disabled");
    }
    if (!task_result_pub_) {
      return;
    }
    std_msgs::msg::Int8 msg;
    msg.data = success ? 1 : -1;
    task_result_pub_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(),
      "Published task result %d on [%s].",
      static_cast<int>(msg.data),
      task_result_topic_.c_str());
  }

  void runSequence() {
    if (mission_controlled_) { runMissionSequence(); return; }
    if (!waitForService<lite_motion_msgs::srv::PlanArmMotion>(
          plan_client_, "plan_arm_motion") ||
        (e3_collision_object_enabled_ &&
         (!waitForService<lite_motion_msgs::srv::AddSceneObject>(
            add_object_client_, "add_scene_object") ||
          !waitForService<lite_motion_msgs::srv::RemoveSceneObject>(
            remove_object_client_, "remove_scene_object") ||
          !waitForService<lite_motion_msgs::srv::SetSceneCollisionAllowance>(
            set_scene_collision_allowance_client_, "set_scene_collision_allowance")))) {
      return;
    }

    if (sentry_mode_) {
      const bool success = runSentrySequence();
      record_tcp_trajectory_.store(false);
      publishReferenceMarkers("sentry_sequence_complete");
      publishTaskResult(success);
      return;
    }

    do {
      if (!waitForTaskStart()) {
        return;
      }
      const bool success = runSequenceOnce();
      record_tcp_trajectory_.store(false);
      publishReferenceMarkers("tcp_trajectory_complete");
      publishTaskResult(success);
      setPhase("waiting");
      if (!success) {
        if (task_start_gate_enabled_) {
          continue;
        }
        return;
      }
      if ((sequence_loop_ || task_start_gate_enabled_) && !stopping()) {
        sleepAfterAction(loop_pause_sec_);
      }
    } while ((sequence_loop_ || task_start_gate_enabled_) && !stopping());
  }

  void replayRecordedSequence() {
    RCLCPP_INFO(
      this->get_logger(),
      "Starting tzb_catch recorded trajectory replay: segments=%zu loop=%s speed_scale=%.3f",
      replay_segments_.size(),
      replay_loop_ ? "true" : "false",
      replay_speed_scale_);

    do {
      arm_state_ = initial_arm_positions_;
      gripper_state_ = gripper_open_positions_;
      e3_attached_to_tcp_ = false;
      publishReferenceMarkers("replay_reset");
      publishE3Marker();
      publishCurrentTheoreticalJointState("replay_reset");

      for (const auto& segment : replay_segments_) {
        if (stopping()) {
          return;
        }

        e3_attached_to_tcp_ = segment.e3_attached_during_segment;
        publishReferenceMarkers("replay_" + segment.stage_name);
        publishE3Marker();

        const auto scaled_traj = scaledTrajectoryForReplay(segment.trajectory);
        if (trajectory_pub_) {
          trajectory_pub_->publish(scaled_traj);
        }
        replayTrajectorySegment(segment);
        sleepAfterAction(replay_segment_pause_sec_);
      }

      if (replay_loop_ && !stopping()) {
        sleepAfterAction(loop_pause_sec_);
      }
    } while (replay_loop_ && !stopping());
  }

  void replayTrajectorySegment(const ReplaySegment& segment) {
    if (segment.trajectory.points.empty()) {
      return;
    }

    const auto segment_start_positions =
      currentNamedJointPositions(segment.trajectory.joint_names);
    const double duration = std::max(0.02, trajectoryDurationSec(segment.trajectory));
    const auto started_at = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(),
      "Replay segment [%s]: joints=%zu duration=%.3f s replay_duration=%.3f s",
      segment.stage_name.c_str(),
      segment.trajectory.joint_names.size(),
      duration,
      duration / replay_speed_scale_);

    while (!stopping()) {
      const double wall_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
      const double trajectory_elapsed = std::min(duration, wall_elapsed * replay_speed_scale_);
      const auto positions = sampleTrajectoryPositions(
        segment.trajectory,
        segment_start_positions,
        trajectory_elapsed);
      applyNamedJointPositions(segment.trajectory.joint_names, positions);
      if (arm_command_publish_during_replay_ &&
          segment.trajectory.joint_names == robot_model_->getPlanningGroup(arm_group_).joint_names) {
        publishArmCommand(segment.trajectory.joint_names, positions);
      }
      publishCurrentTheoreticalJointState("replay_" + segment.stage_name);
      publishReferenceMarkers();
      publishE3Marker();

      if (trajectory_elapsed >= duration) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const auto final_positions = segment.trajectory.points.back().positions;
    applyNamedJointPositions(segment.trajectory.joint_names, final_positions);
    if (arm_command_publish_during_replay_ &&
        segment.trajectory.joint_names == robot_model_->getPlanningGroup(arm_group_).joint_names) {
      publishArmCommand(segment.trajectory.joint_names, final_positions);
    }
    publishCurrentTheoreticalJointState("replay_" + segment.stage_name + "_final");
    publishReferenceMarkers();
    publishE3Marker();
  }

  void sleepAfterAction(double seconds) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!stopping() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void publishReferenceMarkers(const std::string& stage_name = "") {
    if (!ref_marker_pub_) {
      return;
    }
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    int id = 0;
    geometry_msgs::msg::PoseStamped e3_visual_pose;
    if (e3VisualMarkerPose(e3_visual_pose)) {
      appendE3MeshMarker(arr, e3_visual_pose, id);
    }
    geometry_msgs::msg::PoseStamped e3_pose;
    if (currentE3TargetPoseForVisualization(e3_pose)) {
      appendFrameMarkers(arr, e3_pose, "E3_target", id);
    }
    appendFrameMarkers(arr, currentFramePoseStamped(tcp_frame_), tcp_frame_, id);
    if (!pose_estimator_reference_frame_.empty()) {
      try {
        appendFrameMarkers(
          arr,
          currentFramePoseStamped(pose_estimator_reference_frame_),
          pose_estimator_reference_frame_,
          id);
      } catch (const std::exception& e) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Skipping camera reference markers for [%s]: %s",
          pose_estimator_reference_frame_.c_str(),
          e.what());
      }
    }
    geometry_msgs::msg::PoseStamped shadow_camera_pose;
    if (shadowCameraFramePoseStamped(shadow_camera_pose)) {
      appendFrameMarkers(arr, shadow_camera_pose, frozen_camera_frame_id_, id);
    }
    appendTcpTrajectoryMarkers(arr, id);
    ref_marker_pub_->publish(arr);
    if (!stage_name.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Published tzb_catch reference markers at stage [%s].", stage_name.c_str());
    }
  }

  void appendTcpTrajectoryMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    int& id) {
    if (!publish_tcp_trajectory_) {
      return;
    }

    std::vector<geometry_msgs::msg::Point> planned_points;
    std::vector<geometry_msgs::msg::Point> measured_points;
    {
      std::lock_guard<std::mutex> lock(tcp_trajectory_mutex_);
      planned_points = planned_tcp_trajectory_;
      measured_points = measured_tcp_trajectory_;
    }

    const auto append_line = [this, &arr, &id](
      const std::string& marker_namespace,
      const std::vector<geometry_msgs::msg::Point>& points,
      float red,
      float green,
      float blue) {
      if (points.size() < 2) {
        return;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = planning_frame_;
      marker.header.stamp = this->now();
      marker.ns = marker_namespace;
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = tcp_trajectory_line_width_m_;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      marker.color.a = 1.0f;
      marker.points = points;
      arr.markers.push_back(std::move(marker));
    };

    append_line("tcp_planned_trajectory", planned_points, 0.0f, 0.8f, 1.0f);
    append_line("tcp_measured_trajectory", measured_points, 1.0f, 0.45f, 0.0f);
  }

  void appendE3MeshMarker(
    visualization_msgs::msg::MarkerArray& arr,
    const geometry_msgs::msg::PoseStamped& pose,
    int& id) const {
    visualization_msgs::msg::Marker marker;
    marker = makeE3MeshMarker(pose, id++);
    arr.markers.push_back(marker);
  }

  visualization_msgs::msg::Marker makeE3MeshMarker(
    const geometry_msgs::msg::PoseStamped& pose,
    int id = 0) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = this->now();
    marker.ns = "E3_visual";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.frame_locked = e3_attached_to_tcp_;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    return marker;
  }

  void publishE3Marker() {
    if (!e3_marker_pub_) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    if (e3VisualMarkerPose(pose)) {
      e3_marker_pub_->publish(makeE3MeshMarker(pose));
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = this->now();
    marker.ns = "E3_visual";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    e3_marker_pub_->publish(marker);
  }

  bool currentE3TargetPoseForVisualization(geometry_msgs::msg::PoseStamped& out) {
    if (!e3_target_world_pose_.header.frame_id.empty()) {
      out = e3_target_world_pose_;
      return true;
    }
    if (use_yaml_e3_target_) {
      out = e3TargetPoseInPlanningFrame();
      return true;
    }
    return lockedPoseEstimatorTargetInPlanningFrame(out);
  }

  bool e3VisualMarkerPose(geometry_msgs::msg::PoseStamped& out) {
    if (e3_attached_to_tcp_) {
      out.header.frame_id = tcp_frame_;
      out.pose = e3_relative_pose_in_tcp_;
      return true;
    }

    return currentE3TargetPoseForVisualization(out);
  }

  void appendFrameMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    const geometry_msgs::msg::PoseStamped& pose,
    const std::string& ns,
    int& id) const {
    const Eigen::Quaterniond q = quaternionEigenFromMsg(pose.pose.orientation);
    const Eigen::Matrix3d rot = q.toRotationMatrix();
    const Eigen::Vector3d origin(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    const std::array<Eigen::Vector3d, 3> axes = {
      rot * Eigen::Vector3d::UnitX(),
      rot * Eigen::Vector3d::UnitY(),
      rot * Eigen::Vector3d::UnitZ()};

    for (size_t i = 0; i < axes.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = pose.header.frame_id;
      marker.header.stamp = this->now();
      marker.ns = ns + "_axis";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.008;
      marker.scale.y = 0.015;
      marker.scale.z = 0.015;
      marker.color.a = 1.0;
      marker.color.r = (i == 0) ? 1.0f : 0.0f;
      marker.color.g = (i == 1) ? 1.0f : 0.0f;
      marker.color.b = (i == 2) ? 1.0f : 0.0f;
      geometry_msgs::msg::Point p0;
      p0.x = origin.x();
      p0.y = origin.y();
      p0.z = origin.z();
      geometry_msgs::msg::Point p1;
      p1.x = origin.x() + 0.06 * axes[i].x();
      p1.y = origin.y() + 0.06 * axes[i].y();
      p1.z = origin.z() + 0.06 * axes[i].z();
      marker.points.push_back(p0);
      marker.points.push_back(p1);
      arr.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker label;
    label.header.frame_id = pose.header.frame_id;
    label.header.stamp = this->now();
    label.ns = ns + "_label";
    label.id = id++;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose = pose.pose;
    label.pose.position.z += 0.05;
    label.scale.z = 0.03;
    label.color.r = 1.0f;
    label.color.g = 1.0f;
    label.color.b = 1.0f;
    label.color.a = 1.0f;
    label.text = ns;
    arr.markers.push_back(label);
  }

  std::unique_ptr<RobotModel> robot_model_;
  rclcpp::Client<lite_motion_msgs::srv::PlanArmMotion>::SharedPtr plan_client_;
  rclcpp::Client<lite_motion_msgs::srv::AddSceneObject>::SharedPtr add_object_client_;
  rclcpp::Client<lite_motion_msgs::srv::AttachSceneObject>::SharedPtr attach_object_client_;
  rclcpp::Client<lite_motion_msgs::srv::RemoveSceneObject>::SharedPtr remove_object_client_;
  rclcpp::Client<lite_motion_msgs::srv::SetSceneCollisionAllowance>::SharedPtr
    set_scene_collision_allowance_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ref_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr e3_marker_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr task_result_pub_;
  rclcpp::Publisher<arm2nav_msgs::msg::Arm2Nav>::SharedPtr arm2nav_arm_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frozen_camera_pointcloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr task_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr navigation_goal_status_sub_;
  rclcpp::Subscription<arm2nav_msgs::msg::Arm2Nav>::SharedPtr arm2nav_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_estimator_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr reference_marker_timer_;
  rclcpp::TimerBase::SharedPtr frozen_camera_tf_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> frozen_camera_tf_broadcaster_;

  std::thread worker_;
  std::atomic_bool stop_requested_{false};

  // Graceful shutdown state (issue 5).
  std::atomic_bool operation_abort_{false};
  std::atomic_bool shutdown_requested_{false};
  std::atomic_bool shutdown_finished_{false};
  std::atomic_bool emergency_abort_{false};
  std::atomic_int shutdown_exit_code_{0};
  std::atomic<int64_t> shutdown_requested_ns_{0};
  std::mutex worker_mutex_;
  bool worker_spawn_closed_{false};
  std::thread shutdown_thread_;
  std::mutex shutdown_cv_mutex_;
  std::condition_variable shutdown_cv_;
  mutable std::mutex phase_mutex_;
  std::string current_phase_{"waiting"};
  bool home_on_shutdown_{true};
  double shutdown_home_timeout_sec_{30.0};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_hold_pub_;
  rclcpp::TimerBase::SharedPtr shutdown_hold_timer_;

  std::string planning_frame_;
  std::string arm_group_;
  std::string tcp_frame_;
  std::string joint_positions_unit_{"rad"};
  std::string e3_object_id_;
  std::string e3_mesh_resource_;
  std::vector<double> mesh_scale_;
  std::string e3_target_frame_;
  std::array<double, 3> e3_target_xyz_{};
  std::array<double, 3> e3_target_rpy_{};
  bool use_yaml_e3_target_{true};
  bool force_target_roll_{false};
  bool force_target_pitch_{false};
  bool force_target_yaw_{false};
  bool adaptive_rpy_{false};
  bool position_only_target_{false};
  double adaptive_rpy_step_rad_{15.0 * kPi / 180.0};
  double adaptive_rpy_max_rad_{45.0 * kPi / 180.0};
  int adaptive_rpy_max_candidates_{64};
  std::string target_rpy_unit_{"rad"};
  double forced_target_roll_{0.0};
  double forced_target_pitch_{0.0};
  double forced_target_yaw_{0.0};
  double target_yaw_symmetry_deg_{90.0};
  double grasp_ground_z_{std::numeric_limits<double>::quiet_NaN()};
  double grasp_ground_z_max_deviation_{0.08};
  bool grasp_use_visual_z_{false};
  double target_yaw_reference_offset_deg_{0.0};
  std::string pose_estimator_target_topic_{"/pose_estimator/object_pose"};
  double pose_estimator_wait_timeout_sec_{3.0};
  int max_target_attempts_{5};
  double pose_estimator_continuous_required_sec_{2.0};
  double pose_estimator_continuous_max_gap_sec_{0.75};
  std::string pose_estimator_reference_frame_;
  Eigen::Matrix4d pose_estimator_reference_transform_{Eigen::Matrix4d::Identity()};
  bool freeze_camera_pointcloud_enabled_{true};
  bool publish_frozen_camera_tf_{true};
  std::string camera_pointcloud_topic_{"/camera/depth/color/points"};
  std::string frozen_camera_pointcloud_topic_{"/tzb_catch/frozen_observe_camera_points"};
  std::string frozen_camera_frame_id_{"tzb_observe_camera_link"};
  bool apply_reference_transform_to_frozen_frame_{true};
  bool transform_frozen_pointcloud_to_reference_frame_{true};
  bool freeze_reference_from_planned_observe_{true};
  bool camera_frame_from_measured_joints_{false};
  // Joints at the last lock (median over the lock window) and the camera_link FK there;
  // filled on every lock, applied as the frozen reference only when
  // camera_frame_from_measured_joints_ is true.
  std::vector<double> lock_measured_joints_;
  bool lock_measured_joints_valid_{false};
  pinocchio::SE3 lock_measured_camera_in_planning_{pinocchio::SE3::Identity()};
  double mission_calib_observe_j1_{0.0};
  std::vector<double> mission_calib_joints_;
  std::vector<double> observed_reference_joints_;
  double frozen_camera_pointcloud_log_throttle_sec_{2.0};
  bool pointcloud_transform_error_logged_{false};
  bool shadow_camera_pose_error_logged_{false};
  std::chrono::steady_clock::time_point last_frozen_pointcloud_log_time_{};
  Eigen::Matrix3d pose_estimator_axis_conversion_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d target_correction_rpy_deg_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_correction_xyz_{0.0, 0.0, 0.0};
  double target_min_z_{-1.0};
  double target_left_y_threshold_{-1.0};
  double target_left_y_offset_{0.0};
  double grasp_yaw_offset_deg_{0.0};
  std::vector<std::string> gripper_joint_names_;
  std::vector<double> gripper_home_positions_;
  std::vector<double> gripper_open_positions_;
  std::vector<double> gripper_close_positions_;
  int gripper_close_level_{1};
  std::vector<std::string> grasp_collision_links_;
  double grasp_collision_allowed_penetration_m_{-1.0};
  bool e3_collision_object_enabled_{true};
  double joint_planning_timeout_{2.0};
  double cartesian_planning_timeout_{2.0};
  double path_check_step_m_{0.05};
  double cartesian_max_step_m_{0.02};
  int cartesian_min_interpolation_steps_{2};
  int cartesian_refinement_max_steps_{16};
  bool cartesian_use_request_solver_settings_{true};
  bool cartesian_use_sdls_{true};
  int cartesian_ik_max_iterations_{400};
  double cartesian_ik_pos_tolerance_{0.008};
  double cartesian_ik_rot_tolerance_{0.08};
  double cartesian_ik_damping_{1.0e-6};
  double cartesian_ik_alpha_{0.6};
  double cartesian_ik_max_step_norm_{0.5};
  int cartesian_ik_max_seed_count_{32};
  double cartesian_nullspace_gain_{0.0};
  double cartesian_singular_threshold_{0.03};
  double cartesian_branch_jump_max_rad_{0.75};
  double cartesian_branch_jump_norm_max_{1.20};
  double cartesian_waypoint_pos_tolerance_{0.002};
  double cartesian_waypoint_rot_tolerance_{0.05};
  double post_action_wait_sec_{0.05};
  double gripper_settle_sec_{1.0};
  double service_wait_sec_{30.0};
  double response_wait_sec_{30.0};
  std::string e3_marker_topic_{"/tzb_catch/e3_marker"};
  bool publish_e3_marker_{true};
  double reference_marker_republish_hz_{5.0};
  bool publish_tcp_trajectory_{true};
  double tcp_trajectory_line_width_m_{0.006};
  double tcp_trajectory_min_distance_m_{0.002};
  size_t tcp_trajectory_max_points_{10000};
  std::atomic_bool record_tcp_trajectory_{false};
  std::mutex tcp_trajectory_mutex_;
  std::vector<geometry_msgs::msg::Point> planned_tcp_trajectory_;
  std::vector<geometry_msgs::msg::Point> measured_tcp_trajectory_;
  std::string gripper_command_topic_{"/tzb_catch/gripper_command"};
  bool enable_visualization_{true};
  bool visualization_playback_enabled_{true};
  double visualization_playback_speed_scale_{1.0};
  bool sequence_loop_{false};
  double loop_pause_sec_{0.5};
  bool replay_after_complete_{true};
  bool replay_loop_{true};
  double replay_speed_scale_{0.35};
  double replay_segment_pause_sec_{0.15};
  bool sentry_mode_{false};
  int sentry_execution_limit_{1};
  double sentry_trigger_pulse_sec_{0.2};
  std::string arm2nav_state_topic_{"/arm2nav"};
  std::string arm2nav_arm_state_topic_{"/arm2nav/from_arm"};
  double joint_goal_tolerance_rad_{0.01};
  bool arm_command_enabled_{true};
  double arm_command_publish_hz_{100.0};
  double arm_command_speed_scale_{1.0};
  double arm_command_goal_tolerance_rad_{0.005};
  bool arm_command_publish_during_replay_{false};
  std::vector<double> initial_arm_positions_;
  std::vector<double> pre_initial_arm_positions_;
  std::vector<double> carry_tuck_joint_positions_;
  bool staged_yaw_enabled_{true};
  double staged_yaw_min_rad_{0.5};
  std::vector<double> place_arm_positions_;
  std::vector<double> observe_joint_positions_;
  std::vector<double> max_arm_command_velocity_rad_s_;
  std::string arm_state_topic_{"/serial/arm_joint_state"};
  bool arm_feedback_enabled_{true};
  double arm_state_wait_timeout_sec_{2.0};
  double arm_state_max_age_sec_{0.5};
  double arm_motion_feedback_timeout_sec_{30.0};
  double arm_motion_goal_tolerance_rad_{0.05};
  double j6_arrival_tolerance_rad_{-1.0};
  double arm_motion_stable_duration_sec_{0.25};
  bool require_arm_feedback_for_completion_{true};
  // Precise pre-grasp arrival (grasp-approach stage only).
  bool grasp_precise_arrival_enabled_{true};
  double grasp_arrival_tolerance_rad_{0.015};
  double grasp_arrival_tolerance_j6_rad_{0.05};
  double grasp_arrival_stable_sec_{0.5};
  double grasp_arrival_max_speed_rad_s_{0.02};
  double grasp_arrival_timeout_sec_{8.0};
  double grasp_residual_gain_{1.0};
  double grasp_residual_max_rad_{0.10};
  int grasp_residual_iterations_{3};
  double grasp_hover_height_m_{0.0};
  bool grasp_contact_stop_enabled_{true};
  double grasp_residual_lead_max_rad_{0.03};
  double grasp_contact_xy_tol_m_{0.01};
  double grasp_contact_above_m_{0.02};
  double grasp_contact_below_m_{0.005};
  // Why the last precise arrival failed ("" when it succeeded or did not run);
  // ends the grasp attempt without the target re-acquisition retry.
  std::string last_grasp_arrival_failure_;
  std::vector<double> arm_state_;
  std::vector<double> gripper_state_;
  std::mutex arm_state_mutex_;
  std::condition_variable arm_state_cv_;
  std::vector<double> latest_measured_arm_positions_;
  std::vector<double> motion_start_measured_;
  std::string last_stuck_joints_;
  std::string last_deadband_joints_;
  bool last_feedback_stale_{false};
  // The lower controller executed nothing during the last stage (remote
  // controller off / arm not enabled / power): every joint stayed within
  // feedback noise of its start value although a command was streamed.
  bool last_no_motion_{false};
  double no_motion_warning_sec_{2.0};
  // Feedback noise bound for "this joint did not move" (real arm feedback
  // repeats identical floats when stationary) and the smallest commanded travel
  // for which "nothing moved" is a diagnosis rather than a no-op stage.
  static constexpr double kNoMotionNoiseRad = 0.002;
  static constexpr double kNoMotionMinTravelRad = 0.02;
  static constexpr double kNoMotionEarlyWarnTravelRad = 0.05;
  // Command rate of a residual-compensation step (the per-joint velocity limit
  // still applies); 0.10 rad takes 0.4 s.
  static constexpr double kGraspResidualSpeedRadS = 0.25;
  static constexpr const char* kNoMotionReason = "no_motion(check_remote_or_power)";
  static constexpr const char* kNoMotionAdvice =
    "check the remote controller (遥控器) / arm enable / arm power";
  std::string last_plan_failure_;
  bool last_service_timeout_{false};
  static constexpr double kRestrictedResponseWaitSec = 3.0;
  std::atomic_bool arm_command_sent_{false};
  std::atomic_bool gripper_closed_on_object_{false};
  std::atomic_bool restricted_recovery_{false};
  std::atomic_bool shutdown_lease_active_{false};
  std::string shutdown_with_object_{"place"};
  bool have_complete_arm_state_{false};
  std::chrono::steady_clock::time_point latest_arm_state_time_{};
  uint64_t arm_state_sequence_{0};
  bool task_autostart_{false};
  bool mission_controlled_{false}, mission_pending_{false}, mission_success_{false};
  // ---- per-class grasp profiles ----
  std::string target_class_{"blue block"};
  nlohmann::json class_profiles_{nlohmann::json::object()};
  double base_grasp_ground_z_{std::numeric_limits<double>::quiet_NaN()};
  double base_grasp_ground_z_max_deviation_{0.08};
  double base_target_min_z_{-1.0};
  double base_grasp_yaw_offset_deg_{0.0};
  double base_target_yaw_reference_offset_deg_{0.0};
  double base_target_yaw_symmetry_deg_{90.0};
  std::vector<double> base_place_arm_positions_;
  std::vector<double> base_gripper_open_positions_;
  std::vector<double> base_gripper_close_positions_;
  int base_gripper_close_level_{1};
  std::string base_e3_object_id_;
  std::string base_e3_mesh_resource_;
  std::vector<double> base_mesh_scale_;
  bool base_position_only_target_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_class_sub_;
  bool legacy_json_command_{false};
  // Action-server state, all under mission_mutex_.
  std::shared_ptr<ExecuteArmTaskGoalHandle> mission_goal_handle_;
  bool mission_awaiting_handle_{false};
  bool mission_executing_{false};
  bool mission_cancel_requested_{false};
  std::atomic_bool mission_abort_{false};
  rclcpp_action::Server<ExecuteArmTask>::SharedPtr mission_action_server_;
  rclcpp::TimerBase::SharedPtr mission_feedback_timer_;
  double mission_feedback_grace_{2.0};
  std::string mission_session_, mission_phase_{"WAITING"}, mission_operation_, navigation_sensor_frame_;
  uint64_t mission_task_id_{0};
  std::vector<double> mission_expected_center_;
  // self-calibration state (calib_* mission commands)
  double mission_calib_yaw_deg_{0.0};
  std::string mission_result_json_;
  bool calib_hold_after_close_{false};
  geometry_msgs::msg::PoseStamped calib_locked_pose_;
  geometry_msgs::msg::PoseStamped calib_current_hover_;
  std::mutex mission_mutex_;
  std::condition_variable mission_cv_;
  std::deque<std::pair<double,std::vector<double>>> mission_joint_history_;
  std::deque<std::pair<double,Eigen::Vector3d>> mission_centers_;
  std::deque<geometry_msgs::msg::PoseStamped> mission_pending_poses_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
  rclcpp::TimerBase::SharedPtr mission_state_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_center_pub_, navigation_hint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_center_sub_, navigation_hint_sub_, navigation_candidate_sub_;
  bool listen_navigation_goal_status_{true};
  bool task_start_gate_enabled_{false};
  std::string task_start_topic_{"/tzb_catch/start"};
  std::string navigation_goal_status_topic_{"/navigation_goal_status"};
  std::string task_result_topic_{"/tzb_catch/result"};
  std::vector<double> sentry_joint_positions_;
  std::mutex task_trigger_mutex_;
  std::condition_variable task_trigger_cv_;
  bool task_trigger_pending_{false};
  bool navigation_trigger_armed_{true};
  arm2nav_msgs::msg::Arm2Nav latest_arm2nav_state_;
  std::mutex sentry_mutex_;
  std::condition_variable sentry_cv_;
  bool sentry_trigger_pending_{false};
  bool sentry_task_active_{false};
  bool last_arm2nav_arm_trigger_{false};
  std::mutex pose_estimator_mutex_;
  std::condition_variable pose_estimator_cv_;
  geometry_msgs::msg::PoseStamped latest_pose_estimator_target_;
  std::vector<geometry_msgs::msg::PoseStamped> pose_lock_window_;
  double target_lock_max_spread_m_{0.01};
  int pose_lock_retry_count_{0};
  bool have_pose_estimator_target_{false};
  bool pose_estimator_capture_enabled_{false};
  bool pose_estimator_stream_active_{false};
  bool have_frozen_pose_estimator_reference_{false};
  pinocchio::SE3 frozen_pose_estimator_reference_in_planning_{pinocchio::SE3::Identity()};
  std::chrono::steady_clock::time_point pose_estimator_stream_start_time_{};
  std::chrono::steady_clock::time_point pose_estimator_last_msg_time_{};
  geometry_msgs::msg::PoseStamped e3_target_world_pose_;
  geometry_msgs::msg::Pose e3_relative_pose_in_tcp_;
  bool e3_attached_to_tcp_{false};
  std::vector<ReplaySegment> replay_segments_;
};

}  // namespace lite_motion_planner

namespace {

// Async-signal-safe delivery: the handler only writes the signal number to a
// self-pipe; the watcher thread in main() does the actual work.
int g_signal_pipe[2] = {-1, -1};

void onProcessSignal(int signum) {
  const unsigned char byte = static_cast<unsigned char>(signum);
  const int saved_errno = errno;
  (void)!::write(g_signal_pipe[1], &byte, 1);
  errno = saved_errno;
}

void installSignalHandlers() {
  if (::pipe(g_signal_pipe) != 0) {
    throw std::runtime_error("tzb_catch: pipe() for signal delivery failed");
  }
  // A full pipe must never block the signal handler.
  const int flags = ::fcntl(g_signal_pipe[1], F_GETFL, 0);
  ::fcntl(g_signal_pipe[1], F_SETFL, flags | O_NONBLOCK);
  struct sigaction action {};
  action.sa_handler = &onProcessSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  // An SSH drop / closed terminal must behave like SIGTERM, not kill us mid-homing.
  sigaction(SIGHUP, &action, nullptr);
  sigaction(SIGQUIT, &action, nullptr);
  // ros2 launch exits immediately on SIGTERM and closes our stdout pipe; the
  // next log line would otherwise kill the homing with SIGPIPE.
  signal(SIGPIPE, SIG_IGN);
}

// Once the watcher thread is gone nobody reads the pipe: fall back to the
// default dispositions so a hang in rclcpp/DDS teardown can still be killed.
void restoreDefaultSignalHandlers() {
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);
  signal(SIGHUP, SIG_DFL);
  signal(SIGQUIT, SIG_DFL);
}

}  // namespace

int main(int argc, char** argv) {
  // No rclcpp signal handlers: SIGINT/SIGTERM start the graceful shutdown
  // (homing with physical confirmation) instead of tearing the context down.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  installSignalHandlers();

  int exit_code = 0;
  {
    auto node = std::make_shared<lite_motion_planner::TzbCatchNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread signal_watcher([node]() {
      for (;;) {
        unsigned char byte = 0;
        const ssize_t n = ::read(g_signal_pipe[0], &byte, 1);
        if (n < 0 && errno == EINTR) {
          continue;
        }
        if (n <= 0 || byte == 0) {
          return;  // sentinel from main or pipe closed
        }
        const int signum = static_cast<int>(byte);
        if (signum == SIGQUIT) {
          node->requestEmergencyAbort(signum);
        } else {
          node->requestGracefulShutdown(signum);
        }
      }
    });

    // Spin until the shutdown sequence has finished. A hard watchdog forces
    // the exit if the worker hangs past the homing timeout.
    const double watchdog_grace_sec = 5.0;
    while (rclcpp::ok() && !node->shutdownFinished()) {
      executor.spin_once(std::chrono::milliseconds(100));
      if (node->shutdownRequested() &&
          node->secondsSinceShutdownRequest() >
          node->shutdownHomeTimeoutSec() + watchdog_grace_sec) {
        RCLCPP_ERROR(
          node->get_logger(),
          "[SHUTDOWN][WATCHDOG] shutdown sequence did not finish within %.1fs (+%.1fs grace); forcing exit",
          node->shutdownHomeTimeoutSec(), watchdog_grace_sec);
        node->publishShutdownHold(false);
        RCLCPP_ERROR(node->get_logger(), "[SHUTDOWN][END] exit_code=2");
        std::fflush(stdout);
        std::fflush(stderr);
        std::_Exit(2);
      }
    }
    if (node->shutdownFinished()) {
      // Let the 10 Hz lease timer publish hold=false a few times so
      // planning_node / the serial driver release promptly.
      const auto release_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < release_until) {
        executor.spin_once(std::chrono::milliseconds(50));
      }
      exit_code = node->shutdownExitCode();
    } else {
      RCLCPP_ERROR(node->get_logger(), "[SHUTDOWN][END] exit_code=2 (context shut down before homing finished)");
      exit_code = 2;
    }
    node->joinShutdownThread();

    const unsigned char sentinel = 0;
    (void)!::write(g_signal_pipe[1], &sentinel, 1);
    signal_watcher.join();
    restoreDefaultSignalHandlers();
    executor.remove_node(node);
  }
  rclcpp::shutdown();
  return exit_code;
}
