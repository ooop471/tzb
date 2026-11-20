#pragma once
#include <tf2_ros/buffer.h>
#include <arm_task_interfaces/action/execute_arm_task.hpp>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "rover_state/geometry.hpp"
#include "rover_state/mission_budget.hpp"
#include "rover_state/unload_zone.hpp"
#include "rover_state/inflation_profile.hpp"
#include "rover_state/centered_grasp.hpp"
#include "rover_state/footprint_grid.hpp"

namespace rover_state {
class MissionNode : public rclcpp::Node {
 public:
  explicit MissionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void stop();
  bool stopped() const { return !plan_pending_ && !follow_pending_; }

 private:
  friend struct MissionBudgetStateTest;
  friend struct OutdoorNavigationTest;
  friend struct UnloadAlignmentTest;
  friend struct DeadlineUnloadTest;
  friend struct TurnaroundTest;
  using ArmTask = arm_task_interfaces::action::ExecuteArmTask;
  using AH = rclcpp_action::ClientGoalHandle<ArmTask>;
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Plan = nav2_msgs::action::ComputePathToPose;
  using Follow = nav2_msgs::action::FollowPath;
  using PH = rclcpp_action::ClientGoalHandle<Plan>;
  using FH = rclcpp_action::ClientGoalHandle<Follow>;
  enum class State {
    IDLE,
    PREPARING,
    STARTING,
    EXPLORING,
    STOPPING,
    LOCALIZING,
    PLANNING,
    APPROACHING,
    SETTLING,
    READY_TO_GRASP,
    GRASPING,
    RESUMING,
    RECOVERING,
    PAUSED,
    FAULT,
    ENTRY_HANDOFF, ENTRY_PLANNING, ENTRY_NAVIGATING,
    EXIT_HANDOFF, FINAL_PLANNING, FINAL_NAVIGATING, UNLOAD_ALIGN_CHECK, COMPLETED,
    UNLOAD_WAIT_STOP, UNLOAD_OPENING, UNLOAD_DWELL, UNLOAD_STOPPING,
    CENTER_HANDOFF, CENTER_SETTLING, CENTER_OBSERVING, CENTER_REFOLDING,
    ALIGN_TARGET, CENTER_RETURN_WAIT, RESTORE_CORRIDOR,
    TARGET_TRACKING, NUDGING, FACING, TURNAROUND_HANDOFF, TURNING_AROUND
  };
  MissionBudget budget_;
  bool budget_reported_{false}, total_time_reported_{false};
  void tick();
  void transition(State state, const std::string &reason);
  std::string state_name() const;
  void fault(const std::string &reason);
  void recover(const std::string &reason);
  void command_exploration(bool enabled);
  bool follower_idle() const;
  bool follower_running() const;
  bool arm_ready() const;
  bool arm_matches() const;
  bool gate_locked() const;
  bool stationary() const;
  bool inputs_ready() const;
  bool runtime_inputs_ready() const;
  bool aligned() const;
  bool ignored(Point point) const;
  void publish();
  void begin_arm(const std::string &operation);
  void capture_start();
  void start_entry_or_exploration();
  bool entry_pending() const;
  bool ensure_inflation(bool outdoor);
  std::unique_ptr<InflationProfile> inflation_;
  InflationProfile::Radii cave_inflation_;
  double outdoor_inflation_{.37};
  double outdoor_speed_{.30}, cave_speed_{.15};
  geometry_msgs::msg::PoseStamped navigation_target(const std::vector<double> &coordinates,
      const std::string &frame, bool relative) const;
  void odometry(const nav_msgs::msg::Odometry &message);
  void target(const geometry_msgs::msg::PoseStamped &message, bool precise);
  void json_status(const std::string &message, nlohmann::json &out, std::optional<Time> &received);
  void plan();
  void follow(const nav_msgs::msg::Path &path);
  void cancel();
  bool age_ok(const std::optional<Time> &time, double timeout) const;
  double age(Time time) const;
  double stamp() const;
  std::optional<Point> stable_target() const;
  void corridor(const nav_msgs::msg::Path &message);
  bool collision_ready();
  bool pose_clear(Pose pose) const;
  void collect_target(Point point,double stamp,bool precise);
  bool select_centered_target();
  nlohmann::json grasp_diagnostics_;
  Time last_grasp_diagnostic_{};
  void exploration_route(const nav_msgs::msg::Path &message);
  void track_grasp_target();
  void rotate_centered(double yaw,bool restore);
  void return_to_corridor(const std::string &reason,bool mark_target);
  void tick_centered();
  bool centered_state() const;
  bool rotation_state() const;
  void begin_centered_grasp();
  void resume_centered(const std::string &reason);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr entry_pub_, exit_pub_, destination_pub_;

  UnloadZone unload_zone_geometry_;
  Point unload_reference_config_{-.40,0.,0.},unload_reference_world_;
  std::string unload_reference_frame_;
  bool unload_reference_valid_{false};
  void update_unload_reference();
  Pose unload_zone_pose_;
  bool unload_alignment_enabled_{false},unload_zone_valid_{false},unload_alignment_active_{false};
  int unload_alignment_attempts_{0},unload_alignment_max_attempts_{5};
  double unload_alignment_speed_{.05},unload_alignment_timeout_{60};
  double unload_alignment_goal_tolerance_{.08},unload_alignment_path_tolerance_{.08};
  std::optional<Time> unload_alignment_started_;
  bool unload_deadline_enabled_{false},unload_deadline_active_{false};
  double unload_deadline_seconds_{10.},unload_deadline_radius_{1.};
  bool unload_deadline_position_ok() const;
  bool try_deadline_unload();
  bool unload_position_ok() const;
  void check_unload_alignment(const std::string &reason);
  void unload_alignment_tick();
  bool unloading() const;
  void unload_tick();
  void publish_unload(bool active, int direction);
  bool unload_ack(int direction) const;
  bool unload_enabled_{false}, unload_complete_{false}, unload_interrupted_{false};
  State unload_saved_{State::UNLOAD_WAIT_STOP};
  double unload_open_{5}, unload_wait_{5}, unload_elapsed_{0};
  uint64_t unload_id_{0};
  Time unload_last_tick_{Clock::now()}, unload_stop_until_{};
  std::optional<Time> unload_received_;
  nlohmann::json unload_feedback_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr unload_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr unload_sub_;
  State state_{State::IDLE};
  Time entered_{Clock::now()};
  std::string reason_{"Waiting for /mission/start"}, session_;
  std::string world_, planning_frame_, base_, arm_frame_;
  bool calibrated_, explore_enabled_{false}, arm_sent_{false}, shutting_down_{false};
  bool grasp_enabled_{true};
  bool cave_only_{false}, exit_navigation_enabled_{false}, exit_latched_{false}, auto_start_{false};
  std::vector<double> destination_;
  std::string destination_frame_;
  std::string exit_trigger_{"corridor"};
  std::vector<double> exit_position_;
  bool exit_relative_{true};
  double exit_radius_{.35};
  double final_tolerance_{.25}, final_yaw_tolerance_{.25}, final_timeout_{180};
  int final_attempts_{0}, final_max_attempts_{3};
  bool entry_enabled_{false}, entry_arrived_{true}, start_captured_{false};
  bool entry_relative_{false}, final_relative_{false};
  Pose start_pose_;
  std::vector<double> entrance_;
  std::string entrance_frame_;
  double entry_tolerance_{.10}, entry_yaw_tolerance_{.10}, entry_timeout_{180};
  int entry_attempts_{0}, entry_max_attempts_{3};
  bool centered_enabled_{false},centered_active_{false};
  Point grasp_identity_anchor_;
  int grasp_alignment_recoveries_{0};
  int grasp_precise_repositions_{0};
  bool grasp_precise_position_{false};
  bool grasp_precise_approach_{false};
  CenteredGraspParameters centered_parameters_;
  double grid_timeout_{2},centerline_timeout_{2.5},body_rear_{.42},body_half_width_{.308},body_margin_{.01};
  double target_tracking_speed_{.15}, target_deceleration_{.4}, target_tracking_timeout_{60};
  double grasp_lateral_tolerance_{.05};
  double grasp_stop_forward_distance_{.30};
  double grasp_adjust_speed_{.10},grasp_adjust_max_{.30},grasp_adjust_band_{.04},grasp_adjust_used_{0};
  int grasp_adjust_attempts_{0},corridor_restore_attempts_{0};
  double cave_angular_speed_{.35},cave_heading_limit_{.17453292519943295};
  double preview_wait_{2.0};
  bool preview_complete_{false}, preview_seen_{false}, grasp_after_refold_{true};
  std::optional<Time> preview_depth_received_;
  std::optional<Point> preview_target_;
  int preview_searches_{0};
  bool retry_positioned_preview();
  int refold_attempts_{0};
  std::vector<Pose> exploration_route_;
  double centered_localize_timeout_{12}, unverified_target_cooldown_{60};
  std::deque<std::pair<geometry_msgs::msg::PoseStamped,bool>> pending_targets_;
  double rotation_tolerance_{.015},rotation_speed_{.20},rotation_min_speed_{.18},rotation_timeout_{16},corridor_yaw_{0};
  Pose rotation_origin_;
  std::vector<Pose> corridor_line_,locked_corridor_;
  std::optional<Time> corridor_received_,locked_seen_;
  double corridor_stamp_{0};
  struct Track {std::deque<Sample> hints,centres;std::optional<Time> centre_received;Time seen;};
  std::vector<Track> tracks_;
  FootprintGrid global_grid_,local_grid_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr global_map_,cached_global_map_;
  nav2_msgs::msg::Costmap::ConstSharedPtr local_map_,cached_local_map_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr corridor_sub_,exploration_route_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_map_sub_;
  bool plan_pending_{false}, follow_pending_{false}, cancel_requested_{false};
  bool plan_cancel_sent_{false}, follow_cancel_sent_{false};
  std::optional<bool> nav_success_;
  uint64_t generation_{0}, arm_id_{0}, operation_{0};
  std::optional<uint64_t> interrupted_grasp_;
  std::string arm_operation_;
  double front_, gap_, position_tolerance_, yaw_tolerance_, sensor_timeout_, target_timeout_;
  double runtime_grace_;
  double max_sample_gap_;
  double settle_duration_, settle_position_, settle_yaw_, confirmation_, target_spread_,
      association_;
  double max_target_distance_, min_target_height_, max_target_height_, phase_stamp_{0};
  double planning_timeout_, approach_timeout_, grasp_timeout_, cancel_timeout_,
      preparation_timeout_;
  double max_arm_range_, max_arm_z_, min_arm_z_;
  double grasp_near_gap_, grasp_far_gap_, grasp_half_width_;
  int attempts_{0}, max_attempts_;
  Pose robot_, goal_;
  Point locked_;
  std::deque<Sample> odom_history_;
  std::deque<Sample> hints_, centers_;
  std::optional<Time> hint_received_;
  bool approach_by_hint_{false}, observe_locked_{false};
  double observe_max_range_{0.8}, observe_zone_near_{0.30}, observe_zone_far_{0.60}, observe_zone_half_width_{0.22};
  bool hint_target_in_observe_zone();
  std::optional<Time> odom_received_, center_received_, follower_received_, arm_received_,
      gate_received_;
  std::optional<Time> cancel_started_;
  nlohmann::json follower_state_, arm_state_, gate_state_;
  struct Handled {
    Point point;
    Time expires;
  };
  std::vector<Handled> handled_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp_action::Client<Follow>::SharedPtr follower_;
  rclcpp_action::Client<ArmTask>::SharedPtr arm_client_;
  AH::SharedPtr arm_handle_;
  bool arm_goal_submitted_{false}, arm_cancel_requested_{false}, arm_result_complete_{false};
  bool arm_result_success_{false}, recover_prepare_sent_{false};
  std::optional<Clock::time_point> observe_done_;
  double observe_move_timeout_{15.0}, observe_localize_timeout_{4.0}, recover_timeout_{45.0};
  PH::SharedPtr plan_handle_;
  FH::SharedPtr follow_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr explore_pub_, permission_pub_,
      state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotation_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hint_sub_, center_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> status_subs_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr nav_active_client_;
  bool nav_active_{false}, nav_active_pending_{false};
  std::optional<Clock::time_point> nav_active_polled_;
  void poll_nav_active();
  // Grasp alignment only: straight forward/backward correction along the current heading.
  bool try_nudge();
  void start_facing(bool after_observe = false);
  bool facing_after_observe_{false}, facing_checked_{false};
  void facing_tick();
  void dead_end_observation(const std::string &message);
  void clear_dead_end();
  bool check_dead_end();
  void turnaround_tick();
  bool turnaround_pose_clear(Pose pose) const;
  bool turnaround_sweep_clear(double turn) const;
  bool turnaround_enabled_{true},turnaround_hold_{false},turnaround_active_{false},turnaround_done_{false};
  bool turnaround_started_{false},turnaround_rotated_{false},turnaround_reorient_pending_{false};
  std::optional<bool> turnaround_reoriented_;
  double turnaround_confirmation_{10.},turnaround_timeout_{30.},turnaround_speed_{.35};
  double turnaround_progress_{0},turnaround_last_yaw_{0},turnaround_direction_{1};
  double dead_end_first_stamp_{0},dead_end_last_stamp_{0};
  Pose dead_end_anchor_,turnaround_origin_;
  double dead_end_width_{0};
  std::optional<Time> dead_end_since_,dead_end_received_;
  uint64_t turnaround_request_{0};
  std::vector<double> turnaround_footprint_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dead_end_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reorient_client_;
  void nudge_tick();
  Pose nudge_start_; double nudge_distance_{0}, nudge_timeout_{0};
  double nudge_max_{0.30}, nudge_speed_{0.05}, nudge_tolerance_{0.012};
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rover_state
