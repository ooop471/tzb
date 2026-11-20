#include "rover_state/mission_node.hpp"

#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace rover_state {
MissionNode::MissionNode(const rclcpp::NodeOptions &options) : Node("mission_state", options) {
  cave_inflation_={declare_parameter("navigation_inflation.cave_global_radius",.10),
                    declare_parameter("navigation_inflation.cave_local_radius",.10)};
  cave_speed_=declare_parameter("cave_navigation.max_linear_speed",.15);
  if(!std::isfinite(cave_speed_)||cave_speed_<=0)
    throw std::invalid_argument("Cave navigation speed must be finite and positive");
  outdoor_speed_=declare_parameter("navigation_speed.outdoor_max_linear",.30);
  if(!std::isfinite(outdoor_speed_)||outdoor_speed_<=0)
    throw std::invalid_argument("Outdoor navigation speed must be finite and positive");
  outdoor_inflation_=declare_parameter("navigation_inflation.outdoor_radius",.37);
  const double inflation_timeout=declare_parameter("navigation_inflation.switch_timeout",15.);
  const double inflation_settle=declare_parameter("navigation_inflation.settle_time",.20);
  for(double value:{cave_inflation_[0],cave_inflation_[1],outdoor_inflation_,inflation_timeout,inflation_settle})
    if(!std::isfinite(value)||value<=0)throw std::invalid_argument("Invalid navigation inflation profile");
  inflation_=std::make_unique<InflationProfile>(this,inflation_timeout,inflation_settle);
  unload_enabled_=declare_parameter("unload.enabled",false);
  unload_open_=declare_parameter("unload.open_seconds",5.0);
  unload_wait_=declare_parameter("unload.wait_seconds",5.0);
  for(double value : {unload_open_,unload_wait_})
    if(!std::isfinite(value) || value<=0 || value>5.5)
      throw std::invalid_argument("Unload durations must be in (0,5.5] seconds");
  unload_pub_=create_publisher<std_msgs::msg::String>("/chassis/unload_command",1);
  unload_sub_=create_subscription<std_msgs::msg::String>("/chassis/unload_state",1,
    [this](std_msgs::msg::String::ConstSharedPtr m) {
      json_status(m->data,unload_feedback_,unload_received_);
    });
  budget_.total_seconds=declare_parameter("mission_time.total_seconds",600.);
  budget_.grasp_seconds=declare_parameter("mission_time.grasp_cutoff_seconds",360.);
  budget_.max_grasps=declare_parameter("mission_time.max_grasps",3);
  if(!std::isfinite(budget_.total_seconds)||!std::isfinite(budget_.grasp_seconds)||
     budget_.grasp_seconds<=0||budget_.total_seconds<=budget_.grasp_seconds||budget_.max_grasps<=0)
    throw std::invalid_argument("Invalid mission time/count limits");
  unload_alignment_enabled_=declare_parameter("unload_alignment.enabled",false);
  unload_reference_frame_=declare_parameter<std::string>("unload_alignment.reference_frame","base_link");
  unload_reference_config_={declare_parameter("unload_alignment.reference_x",-.40),
    declare_parameter("unload_alignment.reference_y",0.),0.};
  unload_zone_geometry_.reference_offset=unload_reference_config_;
  if(unload_reference_frame_.empty())throw std::invalid_argument("Unload reference frame cannot be empty");
  unload_zone_geometry_.width=declare_parameter("unload_alignment.zone_width",.5);
  unload_zone_geometry_.height=declare_parameter("unload_alignment.zone_height",.5);
  unload_zone_geometry_.margin=declare_parameter("unload_alignment.margin",.05);
  unload_zone_geometry_.validate();
  unload_alignment_speed_=declare_parameter("unload_alignment.speed",.05);
  unload_alignment_timeout_=declare_parameter("unload_alignment.timeout",60.);
  unload_alignment_goal_tolerance_=declare_parameter("unload_alignment.goal_tolerance",.08);
  unload_alignment_path_tolerance_=declare_parameter("unload_alignment.path_tolerance",.08);
  unload_alignment_max_attempts_=declare_parameter("unload_alignment.max_attempts",5);
  for(double v:{unload_alignment_speed_,unload_alignment_timeout_,
      unload_alignment_goal_tolerance_,unload_alignment_path_tolerance_})
    if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Unload alignment limits must be finite and positive");
  if(unload_alignment_speed_>.10 || unload_alignment_max_attempts_<1 ||
     unload_alignment_goal_tolerance_+unload_alignment_path_tolerance_>=
       std::min(unload_zone_geometry_.width,unload_zone_geometry_.height)/2-unload_zone_geometry_.margin)
    throw std::invalid_argument("Unload alignment needs positive clearance beyond planner/controller tolerances");
  unload_deadline_enabled_=declare_parameter("unload_deadline.enabled",false);
  unload_deadline_seconds_=declare_parameter("unload_deadline.remaining_seconds",10.);
  unload_deadline_radius_=declare_parameter("unload_deadline.radius",1.);
  if(!std::isfinite(unload_deadline_seconds_)||unload_deadline_seconds_<=0||unload_deadline_seconds_>=budget_.total_seconds||
     !std::isfinite(unload_deadline_radius_)||unload_deadline_radius_<=0||
     (unload_deadline_enabled_&&!unload_alignment_enabled_))
    throw std::invalid_argument("Invalid unload deadline window/radius or missing bin-reference alignment");
  calibrated_ = declare_parameter("geometry_calibrated", false);
  grasp_enabled_ = declare_parameter("grasp_enabled", true);
  front_ = declare_parameter("front_offset_m", -1.0);
  gap_ = declare_parameter("target_gap_m", 0.30);
  grasp_near_gap_ = declare_parameter("grasp_region_near_gap_m", 0.13);
  grasp_far_gap_ = declare_parameter("grasp_region_far_gap_m", 0.40);
  grasp_half_width_ = declare_parameter("grasp_region_half_width_m", 0.14);
  position_tolerance_ = declare_parameter("alignment_position_tolerance", 0.05);
  yaw_tolerance_ = declare_parameter("alignment_yaw_tolerance", 0.1745329);
  sensor_timeout_ = declare_parameter("input_timeout", 0.6);
  runtime_grace_ = declare_parameter("runtime_input_grace", 2.0);
  target_timeout_ = declare_parameter("target_timeout", 2.5);
  centered_localize_timeout_ = declare_parameter("centered_grasp.localize_timeout", 12.0);
  unverified_target_cooldown_ = declare_parameter("grasp_unverified_target_cooldown", 60.0);
  settle_duration_ = declare_parameter("settle_duration", 0.35);
  settle_position_ = declare_parameter("settle_position_spread", 0.02);
  settle_yaw_ = declare_parameter("settle_yaw_spread", 0.03);
  confirmation_ = declare_parameter("target_confirmation_duration", 0.2);
  max_sample_gap_ = declare_parameter("target_max_sample_gap", 0.5);
  // Precise localisation happens with the arm at the observe pose (camera looks at the grasp area).
  observe_move_timeout_ = declare_parameter("observe_move_timeout", 15.0);
  observe_localize_timeout_ = declare_parameter("observe_localize_timeout", 4.0);
  // RECOVERING must never wait forever for an arm that is stuck in ERROR.
  recover_timeout_ = declare_parameter("recover_timeout", 45.0);
  // Farther than this when confirmed: approach with the far hint first, observe only near the target.
  observe_max_range_ = declare_parameter("observe_max_range", 0.8);
  // Arm-frame box in which the observe pose sees the target: a hint approach that ends (or stalls) with the
  // target inside it goes on to observe instead of being abandoned.
  observe_zone_near_ = declare_parameter("observe_zone_near", 0.30);
  observe_zone_far_ = declare_parameter("observe_zone_far", 0.60);
  observe_zone_half_width_ = declare_parameter("observe_zone_half_width", 0.22);
  nudge_max_ = declare_parameter("grasp_nudge.max_distance", 0.30);
  nudge_speed_ = declare_parameter("grasp_nudge.speed", 0.05);
  nudge_tolerance_ = declare_parameter("grasp_nudge.tolerance", 0.012);
  if (!(nudge_max_ > 0 && nudge_max_ <= 0.30 && nudge_speed_ > 0 && nudge_speed_ <= 0.08 && nudge_tolerance_ > 0))
    throw std::invalid_argument("grasp_nudge limits: max_distance (0,0.30], speed (0,0.08]");
  target_spread_ = declare_parameter("target_position_spread", 0.03);
  association_ = declare_parameter("target_association_distance", 0.25);
  max_target_distance_ = declare_parameter("max_target_distance", 4.0);
  min_target_height_ = declare_parameter("min_target_height", -0.5);
  max_target_height_ = declare_parameter("max_target_height", 1.0);
  max_arm_range_ = declare_parameter("arm_max_target_range", 1.2);
  min_arm_z_ = declare_parameter("arm_min_target_z", -0.5);
  max_arm_z_ = declare_parameter("arm_max_target_z", 0.8);
  planning_timeout_ = declare_parameter("planning_timeout", 8.0);
  approach_timeout_ = declare_parameter("approach_timeout", 60.0);
  grasp_timeout_ = declare_parameter("grasp_timeout", 90.0);
  cancel_timeout_ = declare_parameter("cancel_timeout", 8.0);
  preparation_timeout_ = declare_parameter("preparation_timeout", 45.0);
  max_attempts_ = declare_parameter("max_alignment_attempts", 2);
  world_ = declare_parameter<std::string>("world_frame", "odom_init");
  planning_frame_ = declare_parameter<std::string>("planning_frame", "map");
  base_ = declare_parameter<std::string>("base_frame", "base_frame");
  arm_frame_ = declare_parameter<std::string>("arm_frame", "base_link");
  for (double value :
       {grasp_near_gap_, grasp_far_gap_, grasp_half_width_, gap_, position_tolerance_, yaw_tolerance_, sensor_timeout_, runtime_grace_, target_timeout_,
        settle_duration_, settle_position_, settle_yaw_, confirmation_, max_sample_gap_, target_spread_,
        association_, max_target_distance_, max_arm_range_, planning_timeout_, approach_timeout_,
        grasp_timeout_, cancel_timeout_, preparation_timeout_, recover_timeout_, centered_localize_timeout_, unverified_target_cooldown_})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Mission limits must be positive and finite");
  if (grasp_near_gap_ >= grasp_far_gap_ || gap_ < grasp_near_gap_ || gap_ > grasp_far_gap_)
    throw std::invalid_argument("Stopping gap must be inside the measured grasp region");
  if (!std::isfinite(front_) || (calibrated_ && front_ < 0) || max_attempts_ < 1 ||
      world_.empty() || planning_frame_.empty() || base_.empty() || arm_frame_.empty() ||
      !std::isfinite(min_arm_z_) || !std::isfinite(max_arm_z_) || min_arm_z_ >= max_arm_z_ ||
      !std::isfinite(min_target_height_) || !std::isfinite(max_target_height_) ||
      min_target_height_ >= max_target_height_)
    throw std::invalid_argument("Invalid mission geometry, frames or attempt limit");
  cave_only_ = declare_parameter("cave_navigation_only", false);
  auto_start_ = declare_parameter("autostart", false);
  exit_navigation_enabled_ = declare_parameter("exit_navigation.enabled", false);
  destination_ = declare_parameter<std::vector<double>>("exit_navigation.goal", std::vector<double>{});
  destination_frame_ = declare_parameter<std::string>("exit_navigation.goal_frame", world_);
  final_tolerance_ = declare_parameter("exit_navigation.position_tolerance", .25);
  final_yaw_tolerance_ = declare_parameter("exit_navigation.yaw_tolerance", .25);
  final_timeout_ = declare_parameter("exit_navigation.timeout", 180.0);
  final_max_attempts_ = declare_parameter("exit_navigation.max_attempts", 3);
  if (cave_only_ && !exit_navigation_enabled_)
    throw std::invalid_argument("cave_navigation_only requires exit_navigation.enabled");
  if (exit_navigation_enabled_ && (destination_.size()!=3 || destination_frame_.empty() ||
      !std::all_of(destination_.begin(),destination_.end(),[](double x){return std::isfinite(x);}) ||
      !std::isfinite(final_tolerance_) || final_tolerance_<=0 ||
      !std::isfinite(final_yaw_tolerance_) || final_yaw_tolerance_<=0 ||
      !std::isfinite(final_timeout_) || final_timeout_<=0 || final_max_attempts_<1))
    throw std::invalid_argument("Exit navigation needs a finite [x,y,yaw] goal, frame and valid limits");
  entry_enabled_ = declare_parameter("entry_navigation.enabled", false);
  entry_arrived_ = !entry_enabled_;
  entrance_ = declare_parameter<std::vector<double>>("entry_navigation.goal", std::vector<double>{});
  entrance_frame_ = declare_parameter<std::string>("entry_navigation.goal_frame", world_);
  entry_relative_ = declare_parameter("entry_navigation.relative_to_start", false);
  final_relative_ = declare_parameter("exit_navigation.relative_to_start", false);
  exit_trigger_ = declare_parameter<std::string>("exit_detection.mode", "corridor");
  exit_position_ = declare_parameter<std::vector<double>>("exit_detection.position", std::vector<double>{});
  exit_relative_ = declare_parameter("exit_detection.relative_to_start", true);
  exit_radius_ = declare_parameter("exit_detection.radius", .35);
  if (exit_trigger_ != "corridor" && exit_trigger_ != "position")
    throw std::invalid_argument("exit_detection.mode must be corridor or position");
  if (exit_trigger_ == "position" && (!exit_navigation_enabled_ || exit_position_.size()!=2 ||
      !std::all_of(exit_position_.begin(),exit_position_.end(),[](double x){return std::isfinite(x);}) ||
      !std::isfinite(exit_radius_) || exit_radius_<=0))
    throw std::invalid_argument("Position exit needs exit_navigation enabled, finite exit_detection.position [x,y] and positive radius");
  entry_tolerance_ = declare_parameter("entry_navigation.position_tolerance", .10);
  entry_yaw_tolerance_ = declare_parameter("entry_navigation.yaw_tolerance", .10);
  entry_timeout_ = declare_parameter("entry_navigation.timeout", 180.0);
  entry_max_attempts_ = declare_parameter("entry_navigation.max_attempts", 3);
  if(entry_enabled_ && (entrance_.size()!=3 || entrance_frame_.empty() ||
     !std::all_of(entrance_.begin(),entrance_.end(),[](double x){return std::isfinite(x);}) ||
     !std::isfinite(entry_tolerance_) || entry_tolerance_<=0 ||
     !std::isfinite(entry_yaw_tolerance_) || entry_yaw_tolerance_<=0 ||
     !std::isfinite(entry_timeout_) || entry_timeout_<=0 || entry_max_attempts_<1))
    throw std::invalid_argument("Entry navigation needs a finite [x,y,yaw] goal, frame and valid limits");
  if((entry_enabled_ && entry_relative_ && entrance_frame_!=world_) ||
     (exit_navigation_enabled_ && final_relative_ && destination_frame_!=world_))
    throw std::invalid_argument("Start-relative navigation goals must use world_frame");
  centered_enabled_=declare_parameter("centered_grasp.enabled",false);
  centered_parameters_.front=std::max(0.,front_);centered_parameters_.near_gap=grasp_near_gap_;
  centered_parameters_.far_gap=grasp_far_gap_;centered_parameters_.half_width=grasp_half_width_;
  centered_parameters_.half_angle=declare_parameter("grasp_region_half_angle_degrees",35.)*M_PI/180.;
  cave_angular_speed_=declare_parameter("cave_navigation.max_angular_speed",.35);
  cave_heading_limit_=declare_parameter("cave_navigation.max_heading_degrees",10.)*M_PI/180.;
  if(!std::isfinite(cave_angular_speed_)||cave_angular_speed_<=0||cave_angular_speed_>.8||
     !std::isfinite(cave_heading_limit_)||cave_heading_limit_<=0||cave_heading_limit_>M_PI/4)
    throw std::invalid_argument("Invalid cave turn limits");
  target_tracking_speed_=declare_parameter("centered_grasp.tracking_speed",.15);
  target_deceleration_=declare_parameter("centered_grasp.deceleration",.4);
  grasp_adjust_speed_=declare_parameter("centered_grasp.adjustment_speed",.10);
  grasp_adjust_max_=declare_parameter("centered_grasp.adjustment_max_distance",.30);
  grasp_adjust_band_=declare_parameter("centered_grasp.adjustment_distance_band",.04);
  grasp_lateral_tolerance_=declare_parameter("centered_grasp.lateral_tolerance",.05);
  if(!std::isfinite(grasp_lateral_tolerance_)||grasp_lateral_tolerance_<=0||grasp_lateral_tolerance_>.14)
    throw std::invalid_argument("Invalid grasp lateral centering tolerance");
  if(!std::isfinite(grasp_adjust_speed_)||grasp_adjust_speed_<=0||grasp_adjust_speed_>.15||
     !std::isfinite(grasp_adjust_max_)||grasp_adjust_max_<=0||grasp_adjust_max_>.30||
     !std::isfinite(grasp_adjust_band_)||grasp_adjust_band_<=0||grasp_adjust_band_>.10)
    throw std::invalid_argument("Invalid centered grasp distance adjustment limits");
  preview_wait_=declare_parameter("centered_grasp.preview_wait",.5);
  centered_parameters_.preferred_radius=declare_parameter("centered_grasp.stop_forward_distance",.30);
  target_tracking_timeout_=declare_parameter("centered_grasp.tracking_timeout",60.);
  for(double v:{target_tracking_speed_,target_deceleration_,target_tracking_timeout_,preview_wait_})
    if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid target tracking limits");
  if(target_tracking_speed_>.15)throw std::invalid_argument("Target tracking exceeds cave speed");
  centered_parameters_.margin=declare_parameter("centered_grasp.region_margin",0.0);
  centered_parameters_.maximum_turn=declare_parameter("centered_grasp.max_yaw_degrees",10.)*M_PI/180.;
  centered_parameters_.maximum_offset=declare_parameter("centered_grasp.centre_band",.14);
  grid_timeout_=declare_parameter("centered_grasp.map_timeout",2.0);
  centerline_timeout_=declare_parameter("centered_grasp.reference_timeout",2.5);
  body_rear_=declare_parameter("centered_grasp.body_rear",.42);
  body_half_width_=declare_parameter("centered_grasp.body_half_width",.308);
  body_margin_=declare_parameter("centered_grasp.footprint_margin",.01);
  rotation_tolerance_=declare_parameter("centered_grasp.rotation_tolerance",.015);
  rotation_speed_=declare_parameter("centered_grasp.rotation_speed",.20);
  rotation_timeout_=declare_parameter("centered_grasp.rotation_timeout",16.0);
  rotation_min_speed_=declare_parameter("centered_grasp.rotation_min_speed",.18);
  if(centered_enabled_&&!cave_only_) {
    centered_parameters_.validate();
    if(!calibrated_)throw std::invalid_argument("Centred grasp requires calibrated geometry");
    for(double x:{grid_timeout_,centerline_timeout_,body_rear_,body_half_width_,body_margin_,rotation_tolerance_,rotation_speed_,rotation_min_speed_,rotation_timeout_})
      if(!std::isfinite(x)||x<=0)throw std::invalid_argument("Invalid centred grasp limits");
    if(rotation_speed_>cave_angular_speed_||rotation_min_speed_>rotation_speed_||rotation_tolerance_>.03)throw std::invalid_argument("Centred grasp rotation limits are too large");
  }
  session_ = std::to_string(Clock::now().time_since_epoch().count());
  tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, this, true);
  planner_ = rclcpp_action::create_client<Plan>(this, "/compute_path_to_pose");
  follower_ = rclcpp_action::create_client<Follow>(this, "/follow_path");
  corridor_sub_=create_subscription<nav_msgs::msg::Path>("/explore/corridor_centerline",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr m){corridor(*m);});
  exploration_route_sub_=create_subscription<nav_msgs::msg::Path>("/exploration/executed_path",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr m){exploration_route(*m);});
  global_map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",1,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){global_map_=m;});
  local_map_sub_=create_subscription<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",1,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr m){local_map_=m;});
  explore_pub_ = create_publisher<std_msgs::msg::String>("/exploration/command", 10);
  arm_client_ = rclcpp_action::create_client<ArmTask>(this, "/arm_task/execute");
  nav_active_client_ = create_client<std_srvs::srv::Trigger>("/lifecycle_manager_navigation/is_active");
  rotation_pub_=create_publisher<geometry_msgs::msg::Twist>("/mission/alignment_cmd_vel",rclcpp::SensorDataQoS());
  permission_pub_ = create_publisher<std_msgs::msg::String>("/mission/motion_permission", 10);
  state_pub_ =
      create_publisher<std_msgs::msg::String>("/mission/state", rclcpp::QoS(1).transient_local());
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/approach_goal",
                                                                rclcpp::QoS(1).transient_local());
  entry_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/entry_goal", rclcpp::QoS(1).transient_local());
  exit_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/exit_position", rclcpp::QoS(1).transient_local());
  destination_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/destination", rclcpp::QoS(1).transient_local());
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odometry(*m); });
  hint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/perception/far_target", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { target(*m, false); });
  center_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission/target_center", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { target(*m, true); });
  status_subs_.push_back(create_subscription<std_msgs::msg::String>(
      "/perception/far_target/status",10,[this](std_msgs::msg::String::ConstSharedPtr m) {
        if(state_!=State::CENTER_OBSERVING)return;
        try {
          const auto j=nlohmann::json::parse(m->data);
          const double age=stamp()-j.at("t").get<double>();
          if(std::isfinite(age)&&age>=-.15&&age<=.6&&j.value("valid",false)&&j.value("n_depth",0)>=5)
            preview_depth_received_=Clock::now();
        }catch(const std::exception&){}
      }));
  status_subs_.push_back(create_subscription<std_msgs::msg::String>(
      "/exploration/state", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        json_status(m->data, follower_state_, follower_received_);
      }));
  status_subs_.push_back(create_subscription<std_msgs::msg::String>(
      "/arm_task/state", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        json_status(m->data, arm_state_, arm_received_);
      }));
  status_subs_.push_back(create_subscription<std_msgs::msg::String>(
      "/mission/gate_state", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        json_status(m->data, gate_state_, gate_received_);
      }));
  using Trigger = std_srvs::srv::Trigger;
  services_.push_back(create_service<Trigger>(
      "/mission/start",
      [this](std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> out) {
        if (state_ != State::IDLE || !stopped()) {
          out->message = "Mission is not idle";
          return;
        }
        if (!calibrated_ || front_ < 0) {
          out->message = "Confirm geometry_calibrated and front_offset_m first";
          return;
        }
        if (!inputs_ready() || !age_ok(gate_received_, sensor_timeout_)) {
          out->message = "Waiting for odometry, calibrated TF, Nav2 and velocity gate";
          return;
        }
        capture_start();
        command_exploration(false);
        arm_sent_ = false;
        transition(State::PREPARING, "Waiting for standstill, then preparing the arm");
        out->success = true;
        out->message = "Preparing the configured mission phases";
      }));
  services_.push_back(create_service<Trigger>(
      "/mission/pause",
      [this](std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> out) {
        if (state_ == State::GRASPING) interrupted_grasp_ = arm_id_;
        command_exploration(false);
        cancel();
        transition(State::PAUSED, "Paused; chassis locked");
        out->success = true;
        out->message = "Pause requested; wait for navigation_busy=false and arm readiness";
      }));
  services_.push_back(create_service<Trigger>(
      "/mission/resume",
      [this](std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> out) {
        if (state_ != State::PAUSED || !stopped() || !follower_idle() || !arm_ready() ||
            !inputs_ready() ||
            (interrupted_grasp_ &&
             arm_state_.value("task_id", uint64_t(-1)) != *interrupted_grasp_)) {
          out->message = "Resume requires idle navigation, fresh inputs and physically ready arm";
          return;
        }
        arm_sent_ = false;
        hints_.clear();
        centers_.clear();
        if (interrupted_grasp_ &&
            arm_state_.value("task_id", uint64_t(-1)) == *interrupted_grasp_ &&
            arm_state_.value("success", false))
          handled_.push_back({locked_, Clock::now()+std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(unverified_target_cooldown_))});
        interrupted_grasp_.reset();
        if(unload_interrupted_) {
          const auto saved=unload_saved_;
          transition(saved,"Resuming interrupted unloading; waiting for stopped interlock");
          unload_interrupted_=false;
          out->success=true;
          return;
        }
        transition(centered_active_ ? State::CENTER_RETURN_WAIT : (exit_latched_ ? State::EXIT_HANDOFF : (entry_pending() ? State::ENTRY_HANDOFF : State::PREPARING)),
                   "Resuming the current mission phase after pause");
        out->success = true;
      }));
  services_.push_back(create_service<Trigger>(
      "/mission/reset",
      [this](std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> out) {
        if ((state_ != State::FAULT && state_ != State::PAUSED && state_ != State::IDLE && state_ != State::COMPLETED) ||
            !stopped() || !follower_idle() || (!cave_only_ && (!age_ok(arm_received_, sensor_timeout_) ||
            arm_state_.value("phase", "") == "BUSY"))) {
          out->message = "Reset requires terminal navigation and an idle arm task";
          return;
        }
        if(unload_interrupted_) {
          out->message="Unloading was interrupted; resume to finish unloading before reset";
          return;
        }
        hints_.clear();
        centers_.clear();
        handled_.clear();
        interrupted_grasp_.reset();
        arm_sent_ = false;
        exit_latched_ = false; final_attempts_ = entry_attempts_ = 0;
        entry_arrived_=!entry_enabled_; start_captured_=false;
        unload_zone_valid_=unload_alignment_active_=unload_deadline_active_=false;unload_alignment_attempts_=0;
        unload_alignment_started_.reset();
        budget_.reset();budget_reported_=total_time_reported_=false;
        centered_active_=false;locked_corridor_.clear();tracks_.clear();
        transition(State::IDLE, "Reset; start explicitly to prepare again");
        out->success = true;
      }));
  services_.push_back(create_service<Trigger>(
      "/mission/cancel",
      [this](std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> out) {
        if (state_ == State::GRASPING) interrupted_grasp_ = arm_id_;
        command_exploration(false);
        cancel();
        transition(State::PAUSED, "Canceled; chassis remains locked");
        out->success = true;
        out->message = "Cancellation requested; chassis locked, arm task may still be running";
      }));
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
    tick();
    publish();
  });
}
double MissionNode::age(Time t) const {
  return std::chrono::duration<double>(Clock::now() - t).count();
}
double MissionNode::stamp() const { return now().seconds(); }
bool MissionNode::age_ok(const std::optional<Time> &t, double timeout) const {
  return t && age(*t) <= timeout;
}
std::string MissionNode::state_name() const {
  switch (state_) {
#define S(x)     \
  case State::x: \
    return #x
    S(IDLE);
    S(PREPARING);
    S(STARTING);
    S(EXPLORING); S(TARGET_TRACKING);
    S(STOPPING);
    S(LOCALIZING);
    S(NUDGING);
    S(FACING);
    S(PLANNING);
    S(APPROACHING);
    S(SETTLING);
    S(READY_TO_GRASP);
    S(GRASPING);
    S(RESUMING);
    S(RECOVERING);
    S(PAUSED);
    S(FAULT);
    S(ENTRY_HANDOFF); S(ENTRY_PLANNING); S(ENTRY_NAVIGATING);
    S(UNLOAD_WAIT_STOP); S(UNLOAD_OPENING); S(UNLOAD_DWELL); S(UNLOAD_STOPPING);
    S(EXIT_HANDOFF); S(FINAL_PLANNING); S(FINAL_NAVIGATING); S(UNLOAD_ALIGN_CHECK); S(COMPLETED);
    S(CENTER_HANDOFF); S(CENTER_SETTLING); S(CENTER_OBSERVING); S(CENTER_REFOLDING);
    S(ALIGN_TARGET); S(CENTER_RETURN_WAIT); S(RESTORE_CORRIDOR);
#undef S
  }
  return "FAULT";
}
void MissionNode::transition(State next, const std::string &reason) {
  if(next==State::COMPLETED)budget_.finish(Clock::now());
  if(unloading() && (next==State::PAUSED || next==State::FAULT)) {
    unload_saved_=state_; unload_interrupted_=true;
  }
  if(unloading() && (next==State::PAUSED || next==State::FAULT || next==State::IDLE || next==State::COMPLETED)) {
    ++unload_id_; publish_unload(false,0);
    unload_stop_until_=Clock::now()+std::chrono::milliseconds(500);
  }
  if(next==State::UNLOAD_WAIT_STOP || next==State::UNLOAD_OPENING || next==State::UNLOAD_DWELL ||
     next==State::UNLOAD_STOPPING) {
    ++unload_id_; unload_received_.reset(); unload_last_tick_=Clock::now();
    if(!unload_interrupted_) unload_elapsed_=0;
  }
  if(next==State::CENTER_SETTLING) {
    // Discard unused observations; the fixed world target remains unchanged.
    centers_.clear(); center_received_.reset(); pending_targets_.clear();
  }
  state_ = next;
  reason_ = reason;
  entered_ = Clock::now();
  phase_stamp_ = stamp();
  RCLCPP_INFO(get_logger(), "%s: %s", state_name().c_str(), reason.c_str());
}
void MissionNode::command_exploration(bool enabled) {
  if (enabled != explore_enabled_) {
    ++generation_;
    explore_enabled_ = enabled;
  }
}
void MissionNode::json_status(const std::string &data, nlohmann::json &out,
                              std::optional<Time> &received) {
  try {
    auto j = nlohmann::json::parse(data);
    if (!j.is_object() || !j.at("session").is_string()) return;
    if (&out == &follower_state_) {
      if (!j.at("generation").is_number_unsigned() || !j.at("idle").is_boolean() ||
          !j.at("enabled").is_boolean() || !j.at("route_ready").is_boolean())
        return;
    } else if (&out == &arm_state_) {
      if (!j.at("task_id").is_number_unsigned() || !j.at("phase").is_string() ||
          !j.at("safe_to_drive").is_boolean() || !j.at("success").is_boolean())
        return;
    } else if (&out == &unload_feedback_) {
      if(!j.at("id").is_number_unsigned() || !j.at("direction").is_number_integer() ||
         !j.at("active").is_boolean() || !j.at("tx_ok").is_boolean() ||
         !j.at("lease_ok").is_boolean() || !j.at("interlock").is_boolean() ||
         !j.at("phase_ok").is_boolean()) return;
    } else if (!j.at("allow").is_boolean() || !j.at("zero").is_boolean())
      return;
    const double dt = stamp() - j.at("stamp").get<double>();
    if (!std::isfinite(dt) || dt < -0.15 || dt > sensor_timeout_) return;
    out = std::move(j);
    received = Clock::now();
  } catch (const std::exception &) { /* malformed heartbeats never renew a lease */
  }
}
bool MissionNode::follower_idle() const {
  return age_ok(follower_received_, runtime_grace_) &&
         follower_state_.value("session", "") == session_ &&
         follower_state_.value("generation", uint64_t(-1)) == generation_ &&
         follower_state_.value("idle", false);
}
bool MissionNode::follower_running() const {
  return age_ok(follower_received_, runtime_grace_) &&
         follower_state_.value("session", "") == session_ &&
         follower_state_.value("generation", uint64_t(-1)) == generation_ &&
         follower_state_.value("enabled", false);
}
bool MissionNode::arm_matches() const {
  return age_ok(arm_received_, sensor_timeout_) && arm_state_.value("session", "") == session_ &&
         arm_state_.value("task_id", uint64_t(-1)) == arm_id_;
}
bool MissionNode::arm_ready() const {
  if (cave_only_) return true;
  if (arm_sent_ && arm_matches() &&
      (state_==State::PREPARING || state_==State::GRASPING || state_==State::LOCALIZING ||
       state_==State::RECOVERING || state_==State::CENTER_REFOLDING) && !arm_result_complete_) return false;
  return age_ok(arm_received_, runtime_grace_) && arm_state_.value("session", "") == session_ &&
         arm_state_.value("phase", "") == "READY" && arm_state_.value("safe_to_drive", false);
}
bool MissionNode::gate_locked() const {
  return age_ok(gate_received_, runtime_grace_) && gate_state_.value("session", "") == session_ &&
         !gate_state_.value("allow", true) && gate_state_.value("zero", false);
}
bool MissionNode::inputs_ready() const {
  return age_ok(odom_received_, sensor_timeout_) && planner_->action_server_is_ready() &&
         follower_->action_server_is_ready() &&
         tf_->canTransform(planning_frame_, world_, tf2::TimePointZero) &&
         (!exit_navigation_enabled_ || final_relative_ || tf_->canTransform(world_,destination_frame_,tf2::TimePointZero)) &&
         (!entry_enabled_ || entry_relative_ || tf_->canTransform(world_,entrance_frame_,tf2::TimePointZero)) &&
         (cave_only_ || tf_->canTransform(base_, arm_frame_, tf2::TimePointZero));
}
bool MissionNode::runtime_inputs_ready() const {
  // Runtime gaps are tolerated; startup and final grasp use fresh measurements.
  return age_ok(odom_received_, runtime_grace_) && age_ok(gate_received_, runtime_grace_) &&
         age_ok(follower_received_, runtime_grace_) &&
         tf_->canTransform(planning_frame_, world_, tf2::TimePointZero) &&
         (cave_only_ || tf_->canTransform(base_, arm_frame_, tf2::TimePointZero));
}
void MissionNode::odometry(const nav_msgs::msg::Odometry &m) {
  const double t = rclcpp::Time(m.header.stamp).seconds();
  const auto &q = m.pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  Point p{m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z};
  if (m.header.frame_id != world_ || m.child_frame_id != base_ || !finite(p) ||
      stamp() - t < -0.15 || stamp() - t > sensor_timeout_ || !std::isfinite(norm) ||
      std::abs(norm - 1) > 0.05)
    return;
  Pose value{p, std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
  if (!odom_history_.empty()) {
    const auto &last = odom_history_.back();
    if (t <= last.stamp) return;
    if (t - last.stamp < 0.5 && (distance(value.p, last.pose.p) > 0.5 ||
                                 std::abs(angle(value.yaw - last.pose.yaw)) > 0.5)) {
      odom_history_.clear();
      hints_.clear();
      centers_.clear();
      RCLCPP_WARN(get_logger(),
                  "Odometry discontinuity: refreshing observations without stopping exploration");
    }
  }
  robot_ = value;
  odom_received_ = Clock::now();
  odom_history_.push_back({t, value});
  while (odom_history_.size() > 2 && t - odom_history_[1].stamp >= settle_duration_)
    odom_history_.pop_front();
}
bool MissionNode::stationary() const {
  return age_ok(odom_received_, sensor_timeout_) &&
         settled(std::vector<Sample>(odom_history_.begin(), odom_history_.end()), settle_duration_,
                 settle_position_, settle_yaw_);
}
bool MissionNode::hint_target_in_observe_zone() {
  // freshest estimate of the target: mean of the current hint samples (world frame) or the lock
  Point p = locked_;
  if (!hints_.empty() && age_ok(hint_received_, 1.5)) {
    p = Point{};
    for (const auto &s : hints_) { p.x += s.pose.p.x; p.y += s.pose.p.y; p.z += s.pose.p.z; }
    p.x /= hints_.size(); p.y /= hints_.size(); p.z /= hints_.size();
  }
  try {
    geometry_msgs::msg::PoseStamped w, a;
    w.header.frame_id = world_; w.pose.orientation.w = 1;
    w.pose.position.x = p.x; w.pose.position.y = p.y; w.pose.position.z = p.z;
    tf2::doTransform(w, a, tf_->lookupTransform(arm_frame_, world_, tf2::TimePointZero));
    const auto &v = a.pose.position;
    const bool inside = v.x >= observe_zone_near_ && v.x <= observe_zone_far_ && std::abs(v.y) <= observe_zone_half_width_;
    RCLCPP_INFO(get_logger(), "Hint target in arm frame (%.2f, %.2f): %s the observe zone", v.x, v.y, inside ? "inside" : "outside");
    if (inside) locked_ = p;
    return inside;
  } catch (const tf2::TransformException &) {
    return false;
  }
}
bool MissionNode::try_nudge() {
  if(centered_active_ && aligned() && !grasp_precise_approach_)return false;
  // Only the distance to the target is wrong: the target is inside the lateral band, so a straight move
  // along the heading puts it inside the grasp rectangle. Anything else goes through the planner.
  const auto a = alignment(robot_, locked_, front_);
  if (!std::isfinite(a.gap) || (!centered_active_ && std::abs(a.lateral) > grasp_half_width_)) return false;
  const double lo = grasp_near_gap_ + 0.03, hi = grasp_far_gap_ - 0.03;
  const double want = std::clamp(a.gap, std::min(lo, hi), std::max(lo, hi));
  double d = a.gap - want;   // > 0 forward, < 0 backward
  if(centered_active_) {
    const auto correction=grasp_distance_correction(robot_,locked_,centered_parameters_,
      grasp_precise_position_?std::min(.015,grasp_adjust_band_):grasp_adjust_band_);
    if(!correction || std::abs(*correction)<=nudge_tolerance_ || grasp_adjust_attempts_>=3)return false;
    const double budget=grasp_adjust_max_-grasp_adjust_used_;
    if(budget<=nudge_tolerance_)return false;
    d=std::clamp(*correction,-budget,budget);
  }
  if (std::abs(d) < nudge_tolerance_ || std::abs(d) > nudge_max_) return false;
  if (!collision_ready()) {
    RCLCPP_WARN(get_logger(), "Grasp nudge skipped: collision maps not ready");
    return false;
  }
  const int sweep_steps=std::max(1,int(std::ceil(std::abs(d)/.02)));
  for (int i=0;i<=sweep_steps;++i) {   // include the exact forward/reverse endpoint
    Pose p = robot_;
    const double k = d*i/sweep_steps;
    p.p.x += k * std::cos(robot_.yaw); p.p.y += k * std::sin(robot_.yaw);
    if (!pose_clear(p)) {
      RCLCPP_WARN(get_logger(), "Grasp nudge %.3f m blocked by the costmap at %.3f m", d, k);
      return false;
    }
  }
  nudge_start_ = robot_; nudge_distance_ = d;
  if(centered_active_) {
    ++grasp_adjust_attempts_;grasp_adjust_used_+=std::abs(d);
    grasp_diagnostics_["distance_adjustment"]={{"forward_distance",a.gap},{"move",d},
      {"attempt",grasp_adjust_attempts_},{"distance_budget_used",grasp_adjust_used_}};
  }
  const double speed=centered_active_?grasp_adjust_speed_:nudge_speed_;
  nudge_timeout_ = std::abs(d) / speed + 6.0;
  RCLCPP_INFO(get_logger(), "Grasp nudge: target gap %.3f m (region %.2f-%.2f), lateral %.3f m -> straight %s %.3f m",
              a.gap, grasp_near_gap_, grasp_far_gap_, a.lateral, d > 0 ? "forward" : "backward", std::abs(d));
  transition(State::NUDGING, "Straight grasp-distance correction");
  return true;
}
void MissionNode::nudge_tick() {
  geometry_msgs::msg::Twist command;
  const double c = std::cos(nudge_start_.yaw), s = std::sin(nudge_start_.yaw);
  const double dx = robot_.p.x - nudge_start_.p.x, dy = robot_.p.y - nudge_start_.p.y;
  const double along = c * dx + s * dy, across = -s * dx + c * dy;
  double remaining = nudge_distance_ - along;
  if(centered_active_) {
    const auto correction=grasp_distance_correction(robot_,locked_,centered_parameters_,0.);
    if(!correction) {
      rotation_pub_->publish(command);
      return_to_corridor("Target no longer admits a straight distance correction",true);return;
    }
    // Never exceed the distance reserved for this move, even when a partial
    // correction was selected because the remaining attempt budget is small.
    if(*correction*nudge_distance_<=0)remaining=0.;
    else remaining=std::copysign(std::min(std::abs(*correction),std::max(0.,
      std::abs(nudge_distance_)-along*std::copysign(1.,nudge_distance_))),nudge_distance_);
    grasp_diagnostics_["adjustment_feedback"]={{"forward_distance",alignment(robot_,locked_,front_).gap},
      {"remaining",remaining},{"moved",along}};
  }
  if (std::abs(across) > 0.05 || std::abs(angle(robot_.yaw - nudge_start_.yaw)) > 0.10) {
    rotation_pub_->publish(command);
    recover("Grasp nudge drifted sideways");
    return;
  }
  if (age(entered_) > nudge_timeout_) {
    rotation_pub_->publish(command);
    recover("Grasp nudge timed out");
    return;
  }
  const double speed=centered_active_?grasp_adjust_speed_:nudge_speed_;
  const double stop_distance=centered_active_?std::max(nudge_tolerance_,speed*speed/(2*target_deceleration_)+speed*.05):nudge_tolerance_;
  if (std::abs(remaining) <= stop_distance || remaining * nudge_distance_ < 0) {
    rotation_pub_->publish(command);
    RCLCPP_INFO(get_logger(), "Grasp nudge done: moved %.3f m of %.3f m", along, nudge_distance_);
    transition(centered_active_?State::CENTER_HANDOFF:State::SETTLING, "Grasp distance corrected; checking standstill");
    return;
  }
  const double v = centered_active_?speed:std::min(nudge_speed_, std::max(0.02, 0.8 * std::abs(remaining)));
  if(centered_active_) {
    // Recheck the rear/front stopping corridor against updated maps while moving.
    if(!collision_ready()) {
      rotation_pub_->publish(command);
      return_to_corridor("Collision maps unavailable during grasp distance correction",true);return;
    }
    const double lookahead=std::min(std::abs(remaining),v*v/(2*target_deceleration_)+v*.15+.02);
    const int steps=std::max(1,int(std::ceil(lookahead/.02)));
    for(int i=0;i<=steps;++i) {
      Pose p=robot_;const double d=std::copysign(lookahead*i/steps,remaining);
      p.p.x+=d*std::cos(p.yaw);p.p.y+=d*std::sin(p.yaw);
      if(!pose_clear(p)) {
        rotation_pub_->publish(command);
        return_to_corridor("Grasp distance correction blocked by updated obstacle map",true);return;
      }
    }
  }
  command.linear.x = remaining > 0 ? v : -v;
  rotation_pub_->publish(command);
}
void MissionNode::start_facing(bool after_observe) {
  facing_after_observe_ = after_observe; facing_checked_ = false;
  if (!after_observe && !hints_.empty() && age_ok(hint_received_, 1.5)) locked_ = hints_.back().pose.p;
  if (!after_observe) centers_.clear();
  rotation_origin_ = robot_;
  transition(State::FACING, "Turning in place to face the target before observing");
}
void MissionNode::facing_tick() {
  geometry_msgs::msg::Twist command;
  const double bearing = angle(std::atan2(locked_.y - robot_.p.y, locked_.x - robot_.p.x) - robot_.yaw);
  const auto observe = [this, &command](const char *why) {
    rotation_pub_->publish(command);
    if (facing_after_observe_) {   // precise center already locked: settle, nudge or replan
      if (aligned()) transition(State::SETTLING, "Facing the target; checking standstill before the grasp");
      else if (!try_nudge()) { ++attempts_; plan(); }
      return;
    }
    begin_arm("observe");
    observe_done_.reset();
    transition(State::LOCALIZING, why);
  };
  // the approach may still be braking: take the turn origin only once the rover stands still
  if (age(entered_) < 3.0 && !stationary()) { rotation_origin_ = robot_; rotation_pub_->publish(command); return; }
  // skid steering turns about the wheel centre, not base_frame: allow ~12 cm of base_frame motion
  if (distance(robot_.p, rotation_origin_.p) > 0.12) {
    RCLCPP_WARN(get_logger(), "Facing drift %.3f m", distance(robot_.p, rotation_origin_.p));
    rotation_pub_->publish(command); recover("Position drift while facing the target"); return;
  }
  if (std::abs(bearing) <= 0.05) { observe("Facing the target; observing for the precise center"); return; }
  if (age(entered_) > 12.0) {
    RCLCPP_WARN(get_logger(), "Facing timed out with %.1f deg left; observing anyway", bearing * 180 / M_PI);
    observe("Facing timed out; observing");
    return;
  }
  if (!facing_checked_) {   // check the swept footprint once, from standstill
    facing_checked_ = true;
    if (!collision_ready() || !sweep_clear(robot_, robot_.yaw + bearing, [this](Pose p) { return pose_clear(p); })) {
      RCLCPP_WARN(get_logger(), "In-place turn of %.1f deg not collision-free; observing from here", bearing * 180 / M_PI);
      observe("Turn blocked; observing from the current heading");
      return;
    }
  }
  // the chassis ignores turn rates below ~0.08 rad/s (11-08: 0.05 rad/s commands never moved it)
  // 11-08: the chassis turned only ~0.06 rad/s at a 0.2 rad/s command -> command 0.3-0.5 rad/s
  const double w = std::min(cave_angular_speed_,std::clamp(1.5 * std::abs(bearing), 0.30, 0.50));
  command.angular.z = bearing > 0 ? w : -w;
  rotation_pub_->publish(command);
}
void MissionNode::poll_nav_active() {
  if (nav_active_ || nav_active_pending_) return;
  if (nav_active_polled_ && std::chrono::duration<double>(Clock::now() - *nav_active_polled_).count() < 0.5) return;
  nav_active_polled_ = Clock::now();
  if (!nav_active_client_->service_is_ready()) return;
  nav_active_pending_ = true;
  (void)nav_active_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
      nav_active_pending_ = false;
      try { nav_active_ = f.get()->success; } catch (const std::exception &) {}
    });
}
bool MissionNode::ignored(Point p) const {
  for (const auto &h : handled_)
    if (Clock::now() < h.expires && distance(p, h.point) < association_) return true;
  return false;
}
void MissionNode::target(const geometry_msgs::msg::PoseStamped &m, bool precise) {
  // Accept refinement only while stopped; driving retains the original lock.
  if(centered_enabled_ && centered_active_ && state_!=State::CENTER_OBSERVING &&
     state_!=State::CENTER_SETTLING)return;
  if (state_ != State::EXPLORING && state_ != State::STOPPING && state_ != State::LOCALIZING &&
      state_ != State::PLANNING && state_ != State::APPROACHING && state_ != State::SETTLING &&
      !centered_state())
    return;
  const double t = rclcpp::Time(m.header.stamp).seconds();
  if (stamp() - t > sensor_timeout_ || stamp() - t < -0.15 || m.header.frame_id.empty() ||
      !age_ok(odom_received_, sensor_timeout_)) {
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
      "[GRASP_INPUT] rejected precise=%d frame=%s age=%.3f odometry_fresh=%d",
      precise,m.header.frame_id.c_str(),stamp()-t,age_ok(odom_received_,sensor_timeout_));
    return;
  }
  try {
    geometry_msgs::msg::PoseStamped transformed;
    if (m.header.frame_id == world_)
      transformed = m;
    else
      tf2::doTransform(
          m, transformed,
          tf_->lookupTransform(world_, m.header.frame_id, rclcpp::Time(m.header.stamp)));
    Point p{transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z};
    if (!finite(p) || distance(p, robot_.p) > max_target_distance_ ||
        p.z - robot_.p.z < min_target_height_ || p.z - robot_.p.z > max_target_height_) {
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "[GRASP_INPUT] geometry rejected precise=%d target=(%.3f,%.3f,%.3f) distance=%.3f relative_z=%.3f",
        precise,p.x,p.y,p.z,distance(p,robot_.p),p.z-robot_.p.z);
      return;
    }
    if(state_==State::CENTER_OBSERVING) {
      // Preview uses the detector/depth hint, without waiting for registration.
      // Never replace the frozen world target here.
      if(!precise && arm_result_complete_ && arm_result_success_ && age_ok(preview_depth_received_,.5) && distance(p,locked_)<=.45) {
        preview_seen_=true;
        if(!preview_target_)preview_target_=p;
      }
      return;
    }
    if(centered_active_ && state_==State::CENTER_SETTLING &&
       (distance(p,grasp_identity_anchor_)>association_ ||
        std::abs(p.z-grasp_identity_anchor_.z)>association_ ||
        !stopped() || !stationary()))return;
    if (state_ == State::EXPLORING && ignored(p)) return;
    if(centered_enabled_ && !cave_only_ && state_==State::EXPLORING) {collect_target(p,t,precise);return;}
    if (state_ != State::EXPLORING && distance(p, locked_) > association_) return;
    if(centered_active_)locked_seen_=Clock::now();
    auto &samples = precise ? centers_ : hints_;
    if (!samples.empty() && t <= samples.back().stamp) return;
    if (!samples.empty() &&
        (t - samples.back().stamp > max_sample_gap_ || distance(p, samples.back().pose.p) > association_))
      samples.clear();
    samples.push_back({t, {p, 0}});
    // Preserve the three independent observations required by confirmation,
    // including when camera inference runs near 5 Hz rather than depth's 30 Hz.
    while (samples.size() > 3 && t - samples[1].stamp >= confirmation_) samples.pop_front();
    if (precise) center_received_ = Clock::now(); else hint_received_ = Clock::now();
  } catch (const tf2::TransformException &e) {
    // Camera observations may precede their timestamped odometry TF. Retry
    // without blocking the executor or substituting the latest transform.
    if(pending_targets_.size()>=32)pending_targets_.pop_front();
    pending_targets_.emplace_back(m,precise);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                         "Target TF unavailable at observation time: %s", e.what());
  }
}
std::optional<Point> MissionNode::stable_target() const {
  if (!age_ok(center_received_, sensor_timeout_) || centers_.size() < 3 ||
      centers_.back().stamp - centers_.front().stamp < confirmation_)
    return {};
  Point mean;
  for (const auto &s : centers_) {
    mean.x += s.pose.p.x;
    mean.y += s.pose.p.y;
    mean.z += s.pose.p.z;
  }
  mean.x /= centers_.size();
  mean.y /= centers_.size();
  mean.z /= centers_.size();
  for (const auto &s : centers_)
    if (distance(s.pose.p, mean) > target_spread_ || std::abs(s.pose.p.z - mean.z) > target_spread_)
      return {};
  return mean;
}
bool MissionNode::aligned() const {
  const auto a = alignment(robot_, locked_, front_);
  if(centered_active_)return centered_parameters_.contains(robot_,locked_) &&
      std::abs(angle(robot_.yaw-corridor_yaw_))<=centered_parameters_.maximum_turn+.005;
  // 11-07: the arm plans the grasp from the measured target pose (position + yaw), so any target inside the
  // real grasp region is graspable; the former extra +-5 cm / 10 deg standoff tolerances forced a re-approach
  // whose 3 cm goal checker stalled (field run 23:02).
  return inside_grasp_region(a, grasp_near_gap_, grasp_far_gap_, grasp_half_width_);
}
void MissionNode::capture_start() {
  unload_zone_valid_=unload_alignment_active_=unload_deadline_active_=false;unload_alignment_attempts_=0;
  unload_alignment_started_.reset();
  budget_.start(Clock::now());budget_reported_=total_time_reported_=false;
  unload_complete_=false; unload_interrupted_=false; unload_elapsed_=0;
  start_pose_=robot_;start_captured_=true;exploration_route_.clear();
  centered_active_=false;locked_corridor_.clear();tracks_.clear();
  entry_arrived_=!entry_enabled_;entry_attempts_=final_attempts_=0;exit_latched_=false;
  if(entry_enabled_) entry_pub_->publish(navigation_target(entrance_,entrance_frame_,entry_relative_));
  if(exit_trigger_=="position") exit_pub_->publish(navigation_target(
      {exit_position_[0],exit_position_[1],0.},world_,exit_relative_));
  if(exit_navigation_enabled_) destination_pub_->publish(navigation_target(destination_,destination_frame_,final_relative_));
}
bool MissionNode::entry_pending() const {
  return entry_enabled_ && !entry_arrived_ && start_captured_;
}
bool MissionNode::ensure_inflation(bool outdoor) {
  const auto radii=outdoor?InflationProfile::Radii{outdoor_inflation_,outdoor_inflation_}:cave_inflation_;
  const bool changing=!inflation_->requested()||*inflation_->requested()!=radii;
  if(changing)RCLCPP_INFO(get_logger(),"Switching %s inflation profile: global %.2f m, local %.2f m",
      outdoor?"outdoor":"cave",radii[0],radii[1]);
  const bool was_ready=!changing&&inflation_->ready();
  const bool ready=inflation_->ensure(radii,
      global_map_?rclcpp::Time(global_map_->header.stamp).seconds():-1.,
      local_map_?rclcpp::Time(local_map_->header.stamp).seconds():-1.);
  if(!inflation_->error().empty())fault(inflation_->error());
  if(!was_ready&&ready)RCLCPP_INFO(get_logger(),"Inflation profile ready");
  return ready;
}
void MissionNode::start_entry_or_exploration() {
  hints_.clear();centers_.clear();
  if(entry_pending()) {
    command_exploration(false);
    transition(State::ENTRY_HANDOFF,"Arm ready; navigating to the configured cave entrance before exploration");
  } else {
    if(!ensure_inflation(false))return;
    command_exploration(true);
    transition(State::STARTING,"Cave inflation ready; waiting for a fresh exploration route");
  }
}
geometry_msgs::msg::PoseStamped MissionNode::navigation_target(const std::vector<double> &coordinates,
    const std::string &frame,bool relative) const {
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id=frame;target.header.stamp=now();
  double x=coordinates[0],y=coordinates[1],yaw=coordinates[2],z=robot_.p.z;
  if(relative) {
    if(!start_captured_) throw std::runtime_error("Mission start pose has not been captured");
    x=start_pose_.p.x+std::cos(start_pose_.yaw)*coordinates[0]-std::sin(start_pose_.yaw)*coordinates[1];
    y=start_pose_.p.y+std::sin(start_pose_.yaw)*coordinates[0]+std::cos(start_pose_.yaw)*coordinates[1];
    yaw=angle(start_pose_.yaw+coordinates[2]);z=start_pose_.p.z;
    target.header.frame_id=world_;
  }
  target.pose.position.x=x;target.pose.position.y=y;target.pose.position.z=z;
  target.pose.orientation.z=std::sin(yaw/2);target.pose.orientation.w=std::cos(yaw/2);
  return target;
}
void MissionNode::begin_arm(const std::string &op) {
  ++arm_id_;
  arm_operation_ = op;
  arm_sent_ = true;
  arm_goal_submitted_ = false;
  arm_result_complete_ = false;
  arm_cancel_requested_ = false;
}
void MissionNode::fault(const std::string &why) {
  command_exploration(false);
  cancel();
  if (state_ != State::FAULT) transition(State::FAULT, why);
}
void MissionNode::recover(const std::string &why) {
  if(centered_active_) {return_to_corridor(why,true);return;}
  if (exit_latched_ || entry_pending()) {
    command_exploration(false); cancel();
    transition(exit_latched_ ? State::EXIT_HANDOFF : State::ENTRY_HANDOFF, why);
    return;
  }
  command_exploration(false);
  cancel();
  // Only a target that was reached and still failed (alignment / workspace / grasp) or could not be
  // driven to is ignored for a while; missing a precise center at stop or observe is a range/view problem.
  if (state_ == State::PLANNING || state_ == State::APPROACHING || state_ == State::SETTLING || state_ == State::GRASPING)
    handled_.push_back({locked_, Clock::now() + std::chrono::seconds(30)});
  approach_by_hint_ = false; observe_locked_ = false;
  recover_prepare_sent_ = false;
  transition(State::RECOVERING, why);
}
void MissionNode::plan() {
  if (!stopped() || !follower_idle()) {
    fault("Cannot navigate before all old motion requests terminate");
    return;
  }
  // Entry/final plans must be computed with outdoor inflation already applied.
  if((exit_latched_||entry_pending())&&!ensure_inflation(true))return;
  if(exit_latched_ && (unload_alignment_active_ ? unload_alignment_attempts_>=unload_alignment_max_attempts_ : final_attempts_>=final_max_attempts_)) {
    fault(unload_alignment_active_?"Unload position correction limit reached; unloading blocked":"Final navigation retry limit reached");return;
  }
  const bool entry=entry_pending() && !exit_latched_;
  if(entry && entry_attempts_>=entry_max_attempts_) {fault("Entry navigation retry limit reached");return;}
  if(exit_latched_) {if(unload_alignment_active_)++unload_alignment_attempts_;else ++final_attempts_;}
  if(entry) ++entry_attempts_;
  try {
    if (!exit_latched_ && !entry) goal_ = stopping_pose(robot_, locked_, front_, gap_);
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = world_;
    target.header.stamp = now();
    target.pose.position.x = goal_.p.x;
    target.pose.position.y = goal_.p.y;
    target.pose.position.z = goal_.p.z;
    target.pose.orientation.z = std::sin(goal_.yaw / 2);
    target.pose.orientation.w = std::cos(goal_.yaw / 2);
    if (exit_latched_ || entry) {
      // A coordinate goal uses normal outdoor navigation, independent of grasp hints.
      approach_by_hint_=false;observe_locked_=false;
      target=navigation_target(entry ? entrance_ : destination_,
          entry ? entrance_frame_ : destination_frame_,entry ? entry_relative_ : final_relative_);
      geometry_msgs::msg::PoseStamped world_goal;
      tf2::doTransform(target,world_goal,tf_->lookupTransform(world_,target.header.frame_id,tf2::TimePointZero));
      goal_.p={world_goal.pose.position.x,world_goal.pose.position.y,world_goal.pose.position.z};
      const auto &q=world_goal.pose.orientation;
      goal_.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      if(exit_latched_ && unload_alignment_enabled_) {
        if(!unload_reference_valid_) {fault("Unload reference transform unavailable; unloading blocked");return;}
        unload_zone_pose_=goal_;unload_zone_valid_=true;
        const auto reference=unload_reference_world_;
        const double heading=unload_alignment_active_?robot_.yaw:
          std::atan2(unload_zone_pose_.p.y-reference.y,unload_zone_pose_.p.x-reference.x);
        goal_=unload_zone_geometry_.base_goal(unload_zone_pose_,heading);
        target.header.frame_id=world_;target.header.stamp=now();
        target.pose.position.x=goal_.p.x;target.pose.position.y=goal_.p.y;target.pose.position.z=goal_.p.z;
        target.pose.orientation.x=target.pose.orientation.y=0.;
        target.pose.orientation.z=std::sin(goal_.yaw/2);target.pose.orientation.w=std::cos(goal_.yaw/2);
      }
    } else goal_pub_->publish(target);
    geometry_msgs::msg::PoseStamped mapped;
    tf2::doTransform(target, mapped,
                     tf_->lookupTransform(planning_frame_, target.header.frame_id, tf2::TimePointZero));
    Plan::Goal request;
    request.goal = mapped;
    request.planner_id = "GridBased";
    request.use_start = false;
    plan_pending_ = true;
    cancel_requested_ = false;
    plan_cancel_sent_ = false;
    follow_cancel_sent_ = false;
    nav_success_.reset();
    const auto id = ++operation_;
    transition(entry ? State::ENTRY_PLANNING : (exit_latched_ ? State::FINAL_PLANNING : State::PLANNING),
               entry ? "Planning cave entrance navigation" : (exit_latched_ ? "Planning the configured destination after complete cave exit" : "Planning collision-checked approach"));
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.goal_response_callback = [this, id](PH::SharedPtr handle) {
      if (id != operation_) return;
      plan_handle_ = handle;
      if (!handle) {
        plan_pending_ = false;
        nav_success_ = false;
        return;
      }
      if (cancel_requested_) cancel();
    };
    options.result_callback = [this, id, mapped, entry](const PH::WrappedResult &r) {
      if (id != operation_) return;
      plan_pending_ = false;
      plan_handle_.reset();
      if (cancel_requested_ || (state_ != State::PLANNING && state_ != State::FINAL_PLANNING && state_ != State::ENTRY_PLANNING)) return;
      if (r.code != rclcpp_action::ResultCode::SUCCEEDED || !r.result ||
          r.result->path.poses.size() < 2 || r.result->path.header.frame_id != planning_frame_ ||
          r.result->path.poses.size() > 20000) {
        nav_success_ = false;
        return;
      }
      const auto &end = r.result->path.poses.back().pose.position;
      if (std::hypot(end.x - mapped.pose.position.x, end.y - mapped.pose.position.y) >
          (entry ? entry_tolerance_ : (exit_latched_ ? (unload_alignment_enabled_?unload_alignment_path_tolerance_:final_tolerance_) :
           (approach_by_hint_ ? observe_zone_half_width_ : position_tolerance_)))) {
        // 11-07: a hint approach only has to bring the target into the observe zone (the planner stops
        // short of goals inside inflation by up to its 0.5 m tolerance; 5 cm rejected every such plan).
        RCLCPP_INFO(get_logger(), "Approach plan rejected: path ends %.2f m from the stop pose",
                    std::hypot(end.x - mapped.pose.position.x, end.y - mapped.pose.position.y));
        nav_success_ = false;
        return;
      }
      for (const auto &p : r.result->path.poses)
        if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y)) {
          nav_success_ = false;
          return;
        }
      auto path = r.result->path;
      path.poses.back().pose.orientation = mapped.pose.orientation;
      follow(path);
    };
    (void)planner_->async_send_goal(request, options);
  } catch (const std::exception &e) {
    if (!plan_handle_) plan_pending_ = false;
    recover(std::string("Navigation planning unavailable: ") + e.what());
  }
}
void MissionNode::follow(const nav_msgs::msg::Path &path) {
  const bool entry=entry_pending() && !exit_latched_;
  const bool outdoor=exit_latched_ || entry;
  Follow::Goal goal;
  goal.path = path;
  goal.controller_id = outdoor ? "OutdoorPath" : "ApproachPath";
  if(exit_latched_ && unload_alignment_enabled_ && unload_alignment_active_)goal.controller_id="UnloadAlignPath";
  goal.goal_checker_id = entry ? "entry_goal_checker" : (exit_latched_ ? "exit_goal_checker" : "grasp_goal_checker");
  if(!outdoor && approach_by_hint_)goal.goal_checker_id="hint_goal_checker";  // heading is set by FACING afterwards
  const auto id = operation_;
  follow_pending_ = true;
  nav_success_.reset();
  transition(entry ? State::ENTRY_NAVIGATING : (exit_latched_ ? State::FINAL_NAVIGATING : State::APPROACHING),
             entry ? "Navigating to the cave entrance; exploration remains disabled" : (exit_latched_ ? "Navigating to the configured final destination" : "Following approach path at limited speed"));
  rclcpp_action::Client<Follow>::SendGoalOptions options;
  options.goal_response_callback = [this, id](FH::SharedPtr h) {
    if (id != operation_) return;
    follow_handle_ = h;
    if (!h) {
      follow_pending_ = false;
      nav_success_ = false;
      return;
    }
    if (cancel_requested_) cancel();
  };
  options.result_callback = [this, id](const FH::WrappedResult &r) {
    if (id != operation_) return;
    follow_pending_ = false;
    follow_handle_.reset();
    if (!cancel_requested_) nav_success_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
  };
  try {
    (void)follower_->async_send_goal(goal, options);
  } catch (const std::exception &e) {
    follow_pending_ = false;
    recover(e.what());
  }
}
void MissionNode::cancel() {
  cancel_requested_ = true;
  // Explicit cancellation finishes the current arm segment and restores pre_ini.
  if (arm_sent_ && arm_goal_submitted_ && !arm_result_complete_ && !arm_cancel_requested_) {
    arm_cancel_requested_ = true;
    if(arm_handle_)(void)arm_client_->async_cancel_goal(arm_handle_);
  }
  if ((plan_pending_ || follow_pending_) && !cancel_started_) cancel_started_ = Clock::now();
  try {
    if (plan_handle_ && !plan_cancel_sent_) {
      (void)planner_->async_cancel_goal(plan_handle_);
      plan_cancel_sent_ = true;
    }
    if (follow_handle_ && !follow_cancel_sent_) {
      (void)follower_->async_cancel_goal(follow_handle_);
      follow_cancel_sent_ = true;
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for cancellation: %s",
                         e.what());
  }
}
void MissionNode::tick() {
  update_unload_reference();
  auto pending=std::move(pending_targets_);
  pending_targets_.clear();
  for(const auto &observation:pending)target(observation.first,observation.second);
  if (shutting_down_) {
    cancel();
    return;
  }
  if(state_==State::IDLE && auto_start_ && stopped() && calibrated_ && front_>=0 && inputs_ready() &&
     age_ok(gate_received_,sensor_timeout_)) {
    auto_start_=false;capture_start();
    command_exploration(false);arm_sent_=false;
    transition(State::PREPARING,"Automatic mission startup after valid inputs");
  }
  if (stopped()) cancel_started_.reset();
  if (cancel_started_ && age(*cancel_started_) > cancel_timeout_)
    fault("Navigation cancellation did not reach terminal state");
  // The fallback can recover a navigation/position FAULT only with fresh
  // inputs and a physically safe arm. Operator pause/cancel is never overridden.
  if(state_!=State::IDLE && state_!=State::PAUSED && state_!=State::COMPLETED)
    (void)try_deadline_unload();
  if (state_ == State::IDLE || state_ == State::PAUSED || state_ == State::FAULT || state_ == State::COMPLETED) {
    if (!stopped()) cancel();
    return;
  }
  if(unloading()) {unload_tick();return;}
  const bool closed=budget_.update(Clock::now());
  if(closed && !budget_reported_) {
    budget_reported_=true;
    RCLCPP_INFO(get_logger(),"Grasp phase closed: completed=%d/%d elapsed=%.1f s cutoff=%.1f s; continue exploration to exit and destination",
      budget_.count(),budget_.max_grasps,budget_.elapsed(Clock::now()),budget_.grasp_seconds);
  }
  if(budget_.elapsed(Clock::now())>=budget_.total_seconds && !total_time_reported_) {
    total_time_reported_=true;
    RCLCPP_WARN(get_logger(),"Competition time budget exhausted; continuing safe exit/return without further grasping");
  }
  const bool target_phase=state_==State::STOPPING||state_==State::LOCALIZING||state_==State::PLANNING||
    state_==State::APPROACHING||state_==State::SETTLING||state_==State::READY_TO_GRASP||
    state_==State::GRASPING||state_==State::NUDGING||state_==State::FACING;
  const bool centered_target=centered_active_ && state_!=State::CENTER_RETURN_WAIT &&
    state_!=State::RESTORE_CORRIDOR && state_!=State::CENTER_REFOLDING;
  if(closed && (target_phase||centered_target)) {
    if(centered_active_)return_to_corridor("Grasp time/count limit reached; recovering arm and continuing to cave exit",false);
    else recover("Grasp time/count limit reached; recovering arm and continuing to cave exit");
    return;
  }
  if (!runtime_inputs_ready() ||
      (!cave_only_ && state_ != State::PREPARING && !age_ok(arm_received_, runtime_grace_))) {
    if (state_ == State::PREPARING) {
      if (age(entered_) > preparation_timeout_) fault("Initial preparation could not complete");
    } else if (state_ == State::GRASPING) {
      if (age(entered_) > grasp_timeout_)
        fault("Arm task did not finish; its physical state is unknown");
    } else if (state_ != State::RECOVERING)
      recover("Input gap exceeded grace; waiting to resume the current mission phase");
    return;
  }
  if(!unload_deadline_active_ && unload_alignment_started_ && unload_alignment_enabled_ &&
     (state_==State::UNLOAD_ALIGN_CHECK || state_==State::FINAL_PLANNING || state_==State::FINAL_NAVIGATING || state_==State::EXIT_HANDOFF) &&
     age(*unload_alignment_started_)>unload_alignment_timeout_) {
    fault("Unload position feedback timed out; unloading blocked");return;
  }
  if(centered_state()) {tick_centered();return;}
  switch (state_) {
    case State::PREPARING:
      if (cave_only_ && follower_idle() && gate_locked()) {
        start_entry_or_exploration();
        break;
      }
      if (age(entered_) > preparation_timeout_) {
        fault("Arm preparation or initial standstill timed out");
        break;
      }
      if (!arm_sent_ && follower_idle() && gate_locked() && stationary()) begin_arm("prepare");
      if (arm_sent_ && arm_matches()) {
        if (arm_state_.value("phase", "") == "ERROR") {
          fault("Arm preparation failed");
          break;
        }
        if (arm_ready()) {
          start_entry_or_exploration();
        }
      }
      break;
    case State::STARTING:
    case State::RESUMING:
      if(age(entered_)>recover_timeout_ && arm_ready()) {
        command_exploration(false);command_exploration(true);
        transition(State::RESUMING,"Retrying exploration activation after delayed follower acknowledgement");
      }
      if (follower_running() && arm_ready()) {
        hints_.clear();
        centers_.clear();
        transition(State::EXPLORING, "Exploring; camera target interrupt enabled");
      }
      break;
    case State::EXPLORING: {
      bool at_exit = follower_state_.value("exit_complete",false);
      if (exit_trigger_ == "position" && start_captured_) {
        const auto exit = navigation_target({exit_position_[0],exit_position_[1],0.},world_,exit_relative_);
        at_exit = std::hypot(robot_.p.x-exit.pose.position.x,robot_.p.y-exit.pose.position.y)<=exit_radius_;
      }
      if (exit_navigation_enabled_ && at_exit && (exit_trigger_!="position" || follower_running()) &&
          age_ok(follower_received_,runtime_grace_) &&
          follower_state_.value("session","")==session_ &&
          follower_state_.value("generation",uint64_t(-1))==generation_) {
        exit_latched_=true;command_exploration(false);
        transition(State::EXIT_HANDOFF,exit_trigger_ == "position" ?
            "Configured room exit reached; draining exploration actions" :
            "Ceiling exit confirmed and 1 m forward leg completed; draining exploration actions");
        break;
      }
      // Finish the post-exit 1 m leg before any further visual-target interruption.
      if (budget_.closed() || cave_only_ || follower_state_.value("exit_forward",false) || !follower_running()) break;
      if(centered_enabled_) {select_centered_target();break;}
      if (hints_.size() >= 3 && hints_.back().stamp - hints_.front().stamp >= confirmation_ &&
          stamp() - hints_.back().stamp <= sensor_timeout_) {
        locked_ = hints_.back().pose.p;
        attempts_ = 0;
        command_exploration(false);
        approach_by_hint_ = false; observe_locked_ = false;
        RCLCPP_INFO(get_logger(), "Target confirmed at world (%.2f, %.2f): %.2f m from the rover",
                    locked_.x, locked_.y, std::hypot(locked_.x - robot_.p.x, locked_.y - robot_.p.y));
        transition(State::STOPPING,
                   "Target confirmed; waiting for all exploration actions to terminate");
      }
      break;
    }
    case State::ENTRY_HANDOFF:
      if(!nav_active_) {
        poll_nav_active();
        if(nav_active_) transition(State::ENTRY_HANDOFF, "Nav2 lifecycle active; planning the entrance");
        else if(age(entered_)>60) fault("Nav2 lifecycle did not become active");
        break;
      }
      if(stopped() && follower_idle() && arm_ready()) plan();
      else if(age(entered_)>cancel_timeout_) fault("Entry handoff did not terminate old actions");
      break;
    case State::ENTRY_PLANNING:
    case State::ENTRY_NAVIGATING: {
      const bool timeout=age(entered_)>(state_==State::ENTRY_PLANNING ? planning_timeout_ : entry_timeout_);
      if((nav_success_ && !*nav_success_) || timeout) {
        if(entry_attempts_>=entry_max_attempts_) fault("Entry navigation retry limit reached; exploration not started");
        else recover("Entry navigation failed; draining before retry");
      } else if(state_==State::ENTRY_NAVIGATING && nav_success_ && *nav_success_) {
        const double entry_dist=distance(robot_.p,goal_.p), entry_yaw=std::abs(angle(robot_.yaw-goal_.yaw));
        RCLCPP_INFO(get_logger(),"Entry arrival check: %.2f m from the entrance (tol %.2f), yaw error %.1f deg (tol %.1f)",
                    entry_dist,entry_tolerance_,entry_yaw*180/M_PI,entry_yaw_tolerance_*180/M_PI);
        // 11-07: the entrance only marks where exploration starts; exploration re-orients the rover anyway,
        // so the heading is logged but no longer blocks the hand-off (3 x "inconsistent arrival" -> FAULT).
        // The plan is accepted when its end is within entry_tolerance_ of the entrance and the controller then
        // reports arrival within the same tolerance of that path end, so a consistent arrival lies within
        // 2 x entry_tolerance_ (11-08: 0.27 m vs 0.25 m -> two "inconsistent arrival" -> FAULT).
        if(entry_dist<=2*entry_tolerance_) {
          entry_arrived_=true;start_entry_or_exploration();
        } else if(entry_attempts_>=entry_max_attempts_) fault("Entry arrival result disagrees with measured pose");
        else recover("Replanning entry navigation after inconsistent arrival");
      }
      break;
    }
    case State::EXIT_HANDOFF:
      if (stopped() && follower_idle()) plan();
      else if (age(entered_)>cancel_timeout_) fault("Exit handoff did not terminate exploration actions");
      break;
    case State::FINAL_PLANNING:
    case State::FINAL_NAVIGATING: {
      if(unload_alignment_enabled_ && unload_position_ok()) {
        check_unload_alignment("Bin centre entered unload zone; stopping for position feedback");break;
      }
      const bool timed_out=age(entered_)>(state_==State::FINAL_PLANNING ? planning_timeout_ : final_timeout_);
      const bool failed=(nav_success_ && !*nav_success_) || timed_out;
      if (failed) {
        if(unload_alignment_enabled_ && state_==State::FINAL_NAVIGATING && unload_zone_valid_ &&
           distance(unload_reference_world_,unload_zone_pose_.p)<=1.0) {
          check_unload_alignment("Navigation stopped; checking actual bin position before correction");
        } else if (unload_alignment_active_ ? unload_alignment_attempts_>=unload_alignment_max_attempts_ : final_attempts_>=final_max_attempts_) fault("Final navigation retry limit reached");
        else recover("Final navigation failed or timed out; draining before retry");
      } else if(state_==State::FINAL_NAVIGATING && nav_success_ && *nav_success_) {
        if(unload_alignment_enabled_) {check_unload_alignment("Navigation completed; verifying bin centre, without a heading constraint");break;}
        if(distance(robot_.p,goal_.p)<=final_tolerance_ &&
           std::abs(angle(robot_.yaw-goal_.yaw))<=final_yaw_tolerance_)
          transition(unload_enabled_ ? State::UNLOAD_WAIT_STOP : State::COMPLETED,
              "Final navigation succeeded and measured arrival confirmed");
        else if(final_attempts_>=final_max_attempts_) fault("Navigation result disagrees with measured destination pose");
        else recover("Replanning final navigation after inconsistent arrival result");
      }
      break;
    }
    case State::UNLOAD_ALIGN_CHECK:
      unload_alignment_tick();break;
    case State::STOPPING:
      if (age(entered_) > cancel_timeout_ + settle_duration_ + 2) {
        fault("Exploration cancellation or standstill timed out");
        break;
      }
      if (follower_idle()) {
        const auto p = stable_target();
        if (p && distance(*p, locked_) <= association_) {
          locked_ = *p;
          if (aligned())
            transition(State::SETTLING, "At target distance; checking grasp standstill");
          else {
            ++attempts_;
            plan();
          }
        } else if (std::hypot(locked_.x - robot_.p.x, locked_.y - robot_.p.y) > observe_max_range_) {
          approach_by_hint_ = true;
          RCLCPP_INFO(get_logger(), "Target %.2f m away: approaching with the far hint before observing",
                      std::hypot(locked_.x - robot_.p.x, locked_.y - robot_.p.y));
          ++attempts_;
          plan();
        } else {
          begin_arm("observe");
          observe_done_.reset();
          transition(State::LOCALIZING, "Moving the arm to the observe pose for the precise target center");
        }
      }
      break;
    case State::LOCALIZING: {
      if (arm_operation_ == "observe") {
        if (!arm_result_complete_) {
          if (age(entered_) > observe_move_timeout_) recover("Arm did not reach the observe pose");
          break;
        }
        if (!arm_result_success_) {
          recover("Observe pose move failed");
          break;
        }
        if (!observe_done_) {
          observe_done_ = Clock::now();
          RCLCPP_INFO(get_logger(), "Arm at the observe pose; waiting %.1f s for the precise target center",
                      observe_localize_timeout_);
        }
        if (std::chrono::duration<double>(Clock::now() - *observe_done_).count() > observe_localize_timeout_) {
          recover("Target center not acquired at the observe pose");
          break;
        }
        auto p = stable_target();
        if (!p) break;
        locked_ = *p;
        observe_locked_ = true;
        RCLCPP_INFO(get_logger(), "Precise target center at world (%.2f, %.2f): %.2f m from the rover; folding the arm",
                    locked_.x, locked_.y, std::hypot(locked_.x - robot_.p.x, locked_.y - robot_.p.y));
        begin_arm("prepare");  // the chassis only moves / settles with the arm folded (safe_to_drive)
        break;
      }
      if (!arm_ready()) {
        if (age(entered_) > 2 * observe_move_timeout_) recover("Arm did not fold back after observing");
        break;
      }
      if (aligned()) {
        transition(State::SETTLING, "Already at grasp distance; rechecking alignment");
      } else if (std::abs(alignment(robot_, locked_, front_).lateral) > grasp_half_width_ &&
                 std::abs(alignment(robot_, locked_, front_).yaw) > 0.05) {
        start_facing(true);   // off to the side: turn to it, then nudge the distance
      } else {
        ++attempts_;
        if (!try_nudge()) plan();
      }
      break;
    }
    case State::NUDGING:
      nudge_tick();
      break;
    case State::FACING:
      facing_tick();
      break;
    case State::PLANNING:
      if ((nav_success_ && !*nav_success_) || age(entered_) > planning_timeout_)
        recover("No valid approach plan");
      break;
    case State::APPROACHING:
      if (!follower_idle()) {
        fault("Exploration became active during target approach");
        break;
      }
      if (!approach_by_hint_ && !observe_locked_ && !age_ok(center_received_, target_timeout_)) {
        recover("Target lost during approach");
        break;
      }
      if (age(entered_) > approach_timeout_) {
        recover("Approach timed out");
        break;
      }
      if (nav_success_) {
        if (!*nav_success_) {
          // 11-07 field run: the hint approach stopped with the block 0.48 m ahead / 17 deg right of the arm
          // (observable and graspable) but the tight grasp goal checker + progress checker aborted it.
          if (approach_by_hint_ && hint_target_in_observe_zone()) {
            approach_by_hint_ = false;
            start_facing();
            break;
          }
          recover("Approach controller failed");
          break;
        }
        if (approach_by_hint_) {
          approach_by_hint_ = false;
          start_facing();
          break;
        }
        transition(State::SETTLING,
                   "Approach finished; waiting for standstill and fresh target center");
      }
      break;
    case State::SETTLING: {
      if (age(entered_) > 3) {
        recover("Final target or standstill confirmation timed out");
        break;
      }
      if (!stopped() || !follower_idle() || !gate_locked() || !stationary() || !arm_ready()) break;
      auto p = stable_target();
      if (!p && !observe_locked_) break;
      if (p) locked_ = *p;
      if (!aligned()) {
        if (attempts_ >= max_attempts_) {
          recover("Final alignment outside tolerance");
          break;
        }
        ++attempts_;
        if (!try_nudge()) plan();
        break;
      }
      geometry_msgs::msg::PoseStamped world, local;
      world.header.frame_id = world_;
      world.header.stamp = now();
      world.pose.position.x = locked_.x;
      world.pose.position.y = locked_.y;
      world.pose.position.z = locked_.z;
      world.pose.orientation.w = 1;
      try {
        tf2::doTransform(world, local,
                         tf_->lookupTransform(arm_frame_, world_, tf2::TimePointZero));
        const auto &v = local.pose.position;
        if (std::hypot(v.x, v.y) > max_arm_range_ || v.z < min_arm_z_ || v.z > max_arm_z_) {
          recover("Target outside configured arm workspace");
          break;
        }
      } catch (const tf2::TransformException &) {
        fault("Cannot check target in arm frame");
        break;
      }
      if (!grasp_enabled_) {
        transition(State::READY_TO_GRASP,
                   "Approach-only validation: aligned and stopped; grasp disabled");
        break;
      }
      begin_arm("grasp");
      transition(State::GRASPING,
                 "Chassis locked; waiting for this grasp task result and physical arm return");
      break;
    }
    case State::GRASPING:
      if (age(entered_) > grasp_timeout_) {
        if(centered_active_)return_to_corridor("Grasp timed out; canceling and recovering the arm",true);
        else fault("Grasp timed out; arm may still be extended");
        break;
      }
      if (arm_matches()) {
        if (arm_state_.value("phase", "") == "ERROR") {
          if(centered_active_)return_to_corridor("Grasp failed; recovering the arm before resuming exploration",true);
          else fault("Arm task failed; chassis remains locked");
          break;
        }
        if (arm_ready()) {
          const bool success = arm_state_.value("success", false);
          if(centered_active_ && !success && arm_state_.contains("reposition_target") &&
             arm_state_["reposition_target"].is_object()) {
            if(grasp_precise_repositions_>=2) {
              return_to_corridor("Precise chassis reposition attempts exhausted; arm safely folded",true);break;
            }
            try {
              const auto &j=arm_state_["reposition_target"];
              const auto xyz=j.at("xyz").get<std::vector<double>>();
              if(xyz.size()!=3 || j.at("frame").get<std::string>()!=arm_frame_)
                throw std::runtime_error("Invalid precise reposition frame/point");
              geometry_msgs::msg::PoseStamped in,out;
              in.header.frame_id=arm_frame_;in.pose.orientation.w=1.;
              in.pose.position.x=xyz[0];in.pose.position.y=xyz[1];in.pose.position.z=xyz[2];
              tf2::doTransform(in,out,tf_->lookupTransform(world_,arm_frame_,tf2::TimePointZero));
              Point point{out.pose.position.x,out.pose.position.y,out.pose.position.z};
              if(!finite(point)||distance(point,grasp_identity_anchor_)>.35||
                 std::abs(point.z-grasp_identity_anchor_.z)>.30)
                throw std::runtime_error("Precise observation does not match original target");
              locked_=point;grasp_precise_position_=true;++grasp_precise_repositions_;
              grasp_precise_approach_=alignment(robot_,point,front_).gap>centered_parameters_.preferred_radius+.02;
              hints_.clear();centers_.clear();pending_targets_.clear();
              RCLCPP_INFO(get_logger(),"[GRASP_REPOSITION] precise target world=(%.3f,%.3f,%.3f) forward=%.3f attempt=%d/2; arm folded",
                point.x,point.y,point.z,alignment(robot_,point,front_).gap,grasp_precise_repositions_);
              transition(State::CENTER_HANDOFF,"Precise target needs chassis adjustment; arm folded, rechecking standstill");
            } catch(const std::exception &e) {
              return_to_corridor(std::string("Precise reposition input rejected: ")+e.what(),true);
            }
            break;
          }
          handled_.push_back(
              {locked_, Clock::now() + std::chrono::duration_cast<Clock::duration>(
                  std::chrono::duration<double>(success ? unverified_target_cooldown_ : 30.0))});
          hints_.clear();
          centers_.clear();
          if(centered_active_) {
            return_to_corridor(success ? "Arm sequence completed (object unverified); restoring corridor heading" : "Arm safely recovered; restoring corridor heading",false);
            break;
          }
          command_exploration(true);
          transition(State::RESUMING, success ? "Arm sequence completed (object unverified); continuing exploration"
                                              : "Grasp unsuccessful but arm recovered; skipping "
                                                "target and continuing exploration");
        }
      }
      break;
    case State::RECOVERING: {
      const std::string arm_phase = arm_state_.value("phase", "");
      const bool arm_error = arm_phase == "ERROR";
      // The arm can be left at the observe pose (READY but not drive-safe) or
      // in ERROR (observe/prepare failed). Both must be folded before the
      // chassis may move; without this the mission waited forever.
      if (!cave_only_ && !recover_prepare_sent_ && !arm_ready() && (!arm_sent_ || arm_result_complete_) &&
          age_ok(arm_received_, runtime_grace_) &&
          (arm_error || (arm_phase == "READY" && !arm_state_.value("safe_to_drive", false)))) {
        recover_prepare_sent_ = true;
        begin_arm("prepare");
        break;
      }
      if (age(entered_) > recover_timeout_) {
        fault("Recovery did not complete; the arm is not at a drive-safe pose");
        break;
      }
      if (stopped() && follower_idle() && arm_ready()) {
        hints_.clear();
        centers_.clear();
        command_exploration(true);
        transition(State::RESUMING, "Approach abandoned; arm ready and navigation idle");
      }
      break;
    }
    default:
      break;
  }
}
void MissionNode::update_unload_reference() {
  unload_reference_valid_=false;
  if(!unload_alignment_enabled_)return;
  try {
    geometry_msgs::msg::PointStamped in,out;
    in.header.frame_id=unload_reference_frame_;
    in.point.x=unload_reference_config_.x;in.point.y=unload_reference_config_.y;in.point.z=unload_reference_config_.z;
    tf2::doTransform(in,out,tf_->lookupTransform(base_,unload_reference_frame_,tf2::TimePointZero));
    unload_zone_geometry_.reference_offset={out.point.x,out.point.y,out.point.z};
    // Measure the actual bin point through full TF, including vehicle pitch/roll.
    const auto transform=tf_->lookupTransform(world_,unload_reference_frame_,tf2::TimePointZero);
    if(rclcpp::Time(transform.header.stamp).nanoseconds()!=0 &&
       std::abs(stamp()-rclcpp::Time(transform.header.stamp).seconds())>sensor_timeout_)return;
    tf2::doTransform(in,out,transform);
    unload_reference_world_={out.point.x,out.point.y,out.point.z};
    unload_reference_valid_=finite(unload_zone_geometry_.reference_offset)&&finite(unload_reference_world_);
  } catch(const tf2::TransformException &) {}
}
bool MissionNode::unload_deadline_position_ok() const {
  return exit_latched_ && unload_reference_valid_ && unload_zone_valid_ && age_ok(odom_received_,sensor_timeout_) &&
    distance(unload_reference_world_,unload_zone_pose_.p)<=unload_deadline_radius_;
}
bool MissionNode::try_deadline_unload() {
  if(shutting_down_||state_==State::IDLE||state_==State::PAUSED||state_==State::COMPLETED)return false;
  if(!exit_latched_||!unload_deadline_enabled_||!unload_enabled_||!unload_alignment_enabled_||!start_captured_||
     unload_deadline_active_||unload_interrupted_||
     budget_.remaining(Clock::now())>unload_deadline_seconds_ ||
     !runtime_inputs_ready()||!arm_ready())return false;
  // Refresh the actual configured zone even if the rover has not entered the
  // final-navigation phase; never reuse the last grasp target as a destination.
  try {
    const auto target=navigation_target(destination_,destination_frame_,final_relative_);
    geometry_msgs::msg::PoseStamped world;
    tf2::doTransform(target,world,tf_->lookupTransform(world_,target.header.frame_id,tf2::TimePointZero));
    const auto &q=world.pose.orientation;
    unload_zone_pose_={{world.pose.position.x,world.pose.position.y,world.pose.position.z},
      std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
    unload_zone_valid_=true;
  } catch(const tf2::TransformException &) {return false;}
  if(!unload_deadline_position_ok())return false;
  unload_deadline_active_=true;
  RCLCPP_WARN(get_logger(),"Deadline unloading: %.2f s remaining, bin %.3f m from destination (limit %.2f m); stopping before unloading",
    budget_.remaining(Clock::now()),distance(unload_reference_world_,unload_zone_pose_.p),unload_deadline_radius_);
  if(unloading())return true; // preserve a running unload's phase and elapsed time
  centered_active_=false;
  check_unload_alignment("Deadline fallback after confirmed cave exit: bin within destination radius; stop and unload without precision correction");
  return true;
}
bool MissionNode::unload_position_ok() const {
  return unload_reference_valid_ && unload_zone_valid_ && age_ok(odom_received_,sensor_timeout_) &&
    unload_zone_geometry_.contains_point(unload_reference_world_,unload_zone_pose_);
}
void MissionNode::check_unload_alignment(const std::string &reason) {
  if(!unload_alignment_started_)unload_alignment_started_=Clock::now();
  command_exploration(false);cancel();
  transition(State::UNLOAD_ALIGN_CHECK,reason);
}
void MissionNode::unload_alignment_tick() {
  if(!stopped()||!follower_idle()||!gate_locked()||!arm_ready()) {
    
    if(age(entered_)>preparation_timeout_)fault("Unload alignment waiting for navigation cancellation, zero command or safe arm");
    return;
  }
  if(unload_deadline_active_) {
    if(!unload_deadline_position_ok()) {fault("Deadline unload position/feedback lost; unloading blocked");return;}
    // Skip precision alignment. Navigation cancellation, zero-command,
    // arm readiness, fresh feedback and velocity gate checks above still apply.
    transition(State::UNLOAD_WAIT_STOP,"Deadline fallback position confirmed; unloading within 1 m");
    return;
  }
  if(unload_position_ok()) {
    transition(unload_enabled_?State::UNLOAD_WAIT_STOP:State::COMPLETED,
        "Bin centre inside unload zone; no odometry standstill or heading requirement");
    return;
  }
  
  if(unload_alignment_attempts_>=unload_alignment_max_attempts_) {
    fault("Unload position correction limit reached; unloading blocked");return;
  }
  unload_alignment_active_=true;
  transition(State::EXIT_HANDOFF,"Bin centre outside unload zone; low-speed Nav2 position correction");
}
bool MissionNode::unloading() const {
  return state_==State::UNLOAD_WAIT_STOP || state_==State::UNLOAD_OPENING ||
    state_==State::UNLOAD_DWELL || state_==State::UNLOAD_STOPPING;
}
void MissionNode::publish_unload(bool active,int direction) {
  std_msgs::msg::String m;
  m.data=nlohmann::json({{"session",session_},{"id",unload_id_},{"stamp",stamp()},
    {"active",active},{"direction",direction}}).dump();
  unload_pub_->publish(m);
}
bool MissionNode::unload_ack(int direction) const {
  return age_ok(unload_received_,.2) && unload_feedback_.value("session","")==session_ &&
    unload_feedback_.value("id",uint64_t(-1))==unload_id_ &&
    unload_feedback_.value("direction",99)==direction && unload_feedback_.value("active",false) &&
    unload_feedback_.value("tx_ok",false) && unload_feedback_.value("lease_ok",false) &&
    unload_feedback_.value("interlock",false) && unload_feedback_.value("phase_ok",false);
}
void MissionNode::unload_tick() {
  const auto current=Clock::now();
  const double dt=std::min(.1,std::chrono::duration<double>(current-unload_last_tick_).count());
  unload_last_tick_=current;
  const bool position_safe=!unload_alignment_enabled_||
    (unload_deadline_active_?unload_deadline_position_ok():unload_position_ok());
  const bool safe=stopped() && follower_idle() && gate_locked() && arm_ready() && position_safe;
  if(!safe) {
    publish_unload(true,0);
    if(state_!=State::UNLOAD_WAIT_STOP) fault("Unloading interlock lost; actuator stopped, resume required");
    else if(age(entered_)>10) fault("Unloading waiting for navigation cancellation, zero command, safe arm or position");
    return;
  }
  if(state_==State::UNLOAD_WAIT_STOP) {
    publish_unload(true,0);
    if(unload_ack(0)) transition(State::UNLOAD_OPENING,"Opening collection bin for 5 seconds");
    else if(age(entered_)>5) fault("Unload driver did not acknowledge stopped command");
    return;
  }
  const int direction=state_==State::UNLOAD_OPENING ? -1 : 0;
  publish_unload(true,direction);
  if(unload_ack(direction)) unload_elapsed_+=std::max(0.,dt);
  else if(age(entered_)>1.) {fault("Unload serial transmission/lease not confirmed; resume required");return;}
  const double duration=state_==State::UNLOAD_OPENING ? unload_open_ :
    state_==State::UNLOAD_DWELL ? unload_wait_ : .3;
  if(unload_elapsed_>=duration) {
    if(state_==State::UNLOAD_OPENING) transition(State::UNLOAD_DWELL,"Bin open; waiting for blocks to fall");
    else if(state_==State::UNLOAD_DWELL) transition(State::UNLOAD_STOPPING,"Unloading finished; leaving bin open and actuator stopped");
    else {unload_complete_=true; transition(State::COMPLETED,"Timed unloading complete; bin left open, no physical position feedback");}
  }
}

void MissionNode::publish() {
  if(unloading()) {
    // unload_tick publishes the phase command after checking physical standstill.
  } else if(Clock::now()<unload_stop_until_) publish_unload(false,0);
  std_msgs::msg::String message;
  message.data = nlohmann::json({{"session", session_},
                                 {"generation", generation_},
                                 {"enabled", explore_enabled_},
                                 {"stamp", stamp()}})
                     .dump();
  explore_pub_->publish(message);
  const bool recovering=(state_==State::EXPLORING||state_==State::TARGET_TRACKING) && follower_running() && follower_state_.value("recovering",false);
  const bool backing=recovering && follower_state_.value("recovery_kind","")=="BACKUP";
  const bool nudging = state_ == State::NUDGING;
  const bool facing = state_ == State::FACING;
  const bool outdoor_navigation=state_==State::ENTRY_NAVIGATING||state_==State::FINAL_NAVIGATING;
  const bool unload_correction=state_==State::FINAL_NAVIGATING&&unload_alignment_enabled_&&unload_alignment_active_;
  const bool moving = (state_ == State::EXPLORING || state_ == State::TARGET_TRACKING || state_ == State::APPROACHING || state_ == State::FINAL_NAVIGATING || state_ == State::ENTRY_NAVIGATING || rotation_state() || nudging || facing) &&
                      runtime_inputs_ready() && arm_ready() &&
                      (((state_ == State::EXPLORING || state_ == State::TARGET_TRACKING) && follower_running() &&
                        (follower_state_.value("route_ready", false)||recovering)) ||
                       ((state_ == State::APPROACHING || state_ == State::FINAL_NAVIGATING || state_ == State::ENTRY_NAVIGATING || rotation_state() || nudging || facing) && follower_idle()));
  message.data = nlohmann::json({{"session", session_},
                                 {"stamp", stamp()},
                                 {"rotation_only", rotation_state() || nudging || facing},
                                 {"nudge", nudging},
                                 {"recovery_rotation", recovering && !backing},
                                 {"recovery_backup", backing},
                                 {"allow", moving && (!centered_active_ || state_==State::TARGET_TRACKING || (collision_ready() && pose_clear(robot_)))},
                                 {"max_linear", backing ? .10 : recovering ? 0.0 : state_==State::TARGET_TRACKING ? grasp_approach_speed(alignment(robot_,locked_,front_).gap,centered_parameters_.preferred_radius,target_tracking_speed_) : nudging ? (centered_active_?grasp_adjust_speed_:0.08) : (state_ == State::APPROACHING || rotation_state()) ? 0.10 : unload_correction ? unload_alignment_speed_ : outdoor_navigation ? outdoor_speed_ : cave_speed_},
                                 {"max_angular", unload_correction ? .25 : outdoor_navigation ? .8 : std::min(cave_angular_speed_,recovering ? .20 : ((rotation_state() || nudging) ? rotation_speed_ : cave_angular_speed_))}})
                     .dump();
  if(!outdoor_navigation && (centered_active_||state_==State::EXPLORING||state_==State::TARGET_TRACKING)) {
    const auto &line=centered_active_&&!locked_corridor_.empty()?locked_corridor_:corridor_line_;
    if(const auto reference=corridor_projection(line,robot_.p)) {
      auto permission=nlohmann::json::parse(message.data);
      permission["heading_error"]=angle(robot_.yaw-reference->yaw);
      permission["heading_limit"]=cave_heading_limit_;
      message.data=permission.dump();
    }
  }
  permission_pub_->publish(message);
  if (arm_sent_ && (state_ == State::PREPARING || state_ == State::GRASPING ||
                    state_ == State::LOCALIZING || state_ == State::RECOVERING ||
                    state_==State::CENTER_OBSERVING || state_==State::CENTER_REFOLDING)) {
    nlohmann::json request = {{"session", session_},
                              {"task_id", arm_id_},
                              {"command", arm_operation_},
                              {"stamp", stamp()}};
    if (arm_operation_=="observe") {
      try {
        geometry_msgs::msg::PoseStamped w, b;
        w.header.frame_id = world_;
        w.pose.orientation.w = 1;
        w.pose.position.x = locked_.x;
        w.pose.position.y = locked_.y;
        w.pose.position.z = locked_.z;
        tf2::doTransform(w, b, tf_->lookupTransform(arm_frame_, world_, tf2::TimePointZero));
        request["expected_center"] = {b.pose.position.x, b.pose.position.y, b.pose.position.z};
      } catch (const tf2::TransformException &) {
        fault("Lost arm target transform before trigger");
        return;
      }
    }
    if (!arm_goal_submitted_ && arm_client_->action_server_is_ready()) {
      ArmTask::Goal goal;
      goal.session=session_; goal.task_id=arm_id_; goal.command=arm_operation_;
      goal.expected_center.header.frame_id=arm_frame_;
      goal.expected_center.header.stamp=now();
      if(arm_operation_=="observe") {
        goal.expected_center.point.x=request["expected_center"][0];
        goal.expected_center.point.y=request["expected_center"][1];
        goal.expected_center.point.z=request["expected_center"][2];
      }
      const auto task=arm_id_;
      const auto operation=arm_operation_;
      rclcpp_action::Client<ArmTask>::SendGoalOptions options;
      options.goal_response_callback=[this,task](AH::SharedPtr handle) {
        if(task!=arm_id_) return;
        arm_handle_=handle;
        if(!handle) {
          arm_result_complete_=true;
          if(centered_active_)return_to_corridor("Arm task rejected; recovering and continuing exploration",true);
          else fault("Arm task action rejected the request");
          return;
        }
        if(arm_cancel_requested_ || state_==State::PAUSED || state_==State::FAULT || shutting_down_)
          (void)arm_client_->async_cancel_goal(handle);
      };
      options.result_callback=[this,task,operation](const AH::WrappedResult &r) {
        if(task==arm_id_ && operation=="grasp" && r.code==rclcpp_action::ResultCode::SUCCEEDED && r.result && r.result->success) {
          budget_.completed(task);
        }
        if(task==arm_id_) {
          arm_handle_.reset();
          arm_result_complete_=true;
          arm_result_success_=r.result && r.result->success;
          if(r.result && r.result->phase=="ERROR" &&
              (state_==State::PREPARING || state_==State::GRASPING)) {
            if(centered_active_)return_to_corridor(r.result->message,true);
            else fault(r.result->message);
          }
        }
        // Motion readiness comes from the bridge heartbeat, including measured recovery.
      };
      arm_goal_submitted_=true;
      try { (void)arm_client_->async_send_goal(goal,options); }
      catch(const std::exception &e) { fault(std::string("Arm action unavailable: ")+e.what()); }
    }
  }
  const auto a = alignment(robot_, locked_, std::max(0.0, front_));
  message.data = nlohmann::json({{"unload_complete",unload_complete_},{"unload_elapsed",unload_elapsed_},
                                 {"unload_physical_confirmation",false},{"unload_interrupted",unload_interrupted_},
                                 {"unload_deadline_enabled",unload_deadline_enabled_},
                                 {"unload_deadline_active",unload_deadline_active_},
                                 {"unload_deadline_radius",unload_deadline_radius_},
                                 {"unload_deadline_remaining_seconds",unload_deadline_seconds_},
                                 {"unload_deadline_position_ok",unload_deadline_position_ok()},
                                 {"unload_alignment_enabled",unload_alignment_enabled_},
                                 {"unload_position_ok",unload_position_ok()},
                                 {"unload_alignment_attempts",unload_alignment_attempts_},
                                 {"unload_reference_frame",unload_reference_frame_},
                                 {"unload_reference_transform_valid",unload_reference_valid_},
                                 {"unload_reference_offset_base_frame",{unload_zone_geometry_.reference_offset.x,unload_zone_geometry_.reference_offset.y}},
                                 {"unload_reference_world",{unload_reference_world_.x,unload_reference_world_.y}},
                                 {"unload_zone_error",{unload_zone_geometry_.point_error(unload_reference_world_,unload_zone_pose_).x,unload_zone_geometry_.point_error(unload_reference_world_,unload_zone_pose_).y}},
                                 {"unload_zone_centre",{unload_zone_pose_.p.x,unload_zone_pose_.p.y}},
                                 {"unload_heading_required",false},
                                 {"unload_position_source","odometry_tf"},
                                 {"mission_elapsed_seconds",budget_.elapsed(Clock::now())},
                                 {"mission_remaining_seconds",budget_.remaining(Clock::now())},
                                 {"mission_total_seconds",budget_.total_seconds},
                                 {"grasp_cutoff_seconds",budget_.grasp_seconds},
                                 {"grasp_completed_count",budget_.count()},
                                 {"grasp_max_count",budget_.max_grasps},
                                 {"grasp_phase_closed",budget_.update(Clock::now())},
                                 {"state", state_name()},
                                 {"reason", reason_},
                                 {"stamp", stamp()},
                                 {"session", session_},
                                 {"navigation_busy", !stopped() || !follower_idle()},
                                 {"exit_complete", exit_latched_},
                                 {"entry_arrived", entry_arrived_},
                                 {"inflation_ready", inflation_->ready()},
                                 {"inflation_global_radius", inflation_->requested()?(*inflation_->requested())[0]:0.},
                                 {"inflation_local_radius", inflation_->requested()?(*inflation_->requested())[1]:0.},
                                 {"centered_grasp", centered_active_},
                                 {"target_position_locked", centered_active_},
                                 {"preview_seen",preview_seen_},
                                 {"preview_complete",preview_complete_},
                                 {"locked_target_world", {locked_.x,locked_.y,locked_.z}},
                                 {"locked_target_frame",world_},
                                 {"grasp_diagnostics",grasp_diagnostics_},
                                 {"grasp_region", {{"shape","annular_sector"},{"frame",base_},{"front_offset",front_},
                                   {"inner_radius",grasp_near_gap_},{"outer_radius",grasp_far_gap_},
                                   {"half_angle",centered_parameters_.half_angle}}},
                                 {"corridor_yaw", corridor_yaw_},
                                 {"entry_navigation_attempts", entry_attempts_},
                                 {"final_navigation_attempts", final_attempts_},
                                 {"arm_task_id", arm_id_},
                                 {"grasp_physical_confirmation", false},
                                 {"grasp_execution","standalone_launch"},
                                 {"gap_m", a.gap},
                                 {"lateral_m", a.lateral},
                                 {"motion_allowed", moving},
                                 {"stationary", stationary()}})
                     .dump();
  state_pub_->publish(message);
}
void MissionNode::stop() {
  shutting_down_ = true;
  command_exploration(false);
  cancel();
  transition(State::PAUSED, "Shutting down");
  publish();
}
}  // namespace rover_state
