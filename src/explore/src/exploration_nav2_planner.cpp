// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#include "rover_explore/exploration_nav2_planner.hpp"

#include <cmath>
#include "rover_explore/corridor_geometry.hpp"
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rover_explore
{
namespace
{
bool finite(const geometry_msgs::msg::Point &p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(a.x-b.x, a.y-b.y);
}
double age(std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now()-time).count();
}
} // namespace

ExplorationNav2Planner::ExplorationNav2Planner(const rclcpp::NodeOptions &options)
    : Node("exploration_nav2_planner", options)
{
  corridor_enabled_ = declare_parameter("corridor_enabled", false);
  input_timeout_ = declare_parameter("input_timeout", 2.0);
  replan_interval_ = declare_parameter("replan_interval", 1.0);
  planning_timeout_ = declare_parameter("planning_timeout", 4.0);
  retry_delay_ = declare_parameter("retry_delay", 0.5);
  target_update_distance_ = declare_parameter("target_update_distance", 0.15);
  endpoint_tolerance_ = declare_parameter("endpoint_tolerance", 0.6);
  start_tolerance_ = declare_parameter("start_tolerance", 0.8);
  input_frame_ = declare_parameter<std::string>("input_frame", "odom_init");
  planning_frame_ = declare_parameter<std::string>("planning_frame", "map");
  planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
  for (double value : {input_timeout_, replan_interval_, planning_timeout_, retry_delay_,
                       target_update_distance_, endpoint_tolerance_, start_tolerance_})
  {
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Planner intervals and tolerances must be finite and positive");
  }
  if (input_frame_.empty() || planning_frame_.empty() || planner_id_.empty())
    throw std::invalid_argument("Planner frames and planner_id must not be empty");
  tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, this, false);
  client_ = rclcpp_action::create_client<Plan>(this, declare_parameter<std::string>("planning_action", "/compute_path_through_poses"));
  centerline_sub_=create_subscription<nav_msgs::msg::Path>("/explore/corridor_centerline",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr m){if(m->header.frame_id==input_frame_)centerline_=*m;});
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/explore/control_path", 2);
  target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/explore/waypoint", 2,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) { on_target(*message); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_odometry(*message); });
  enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/explore/enabled", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (enabled_ != message->data) target_input_ = {};
        enabled_ = message->data;
        if (!enabled_) cancel();
      });
  complete_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/explore/coverage_complete", 2,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (complete_ != message->data) target_input_ = {};
        complete_ = message->data;
        if (complete_) cancel();
      });
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { tick(); });
  RCLCPP_INFO(get_logger(), "Exploration path source: %s",
      corridor_enabled_ ? "Nav2 path to observed center goal; soft corridor guidance" : "Nav2 global planner");
}

bool ExplorationNav2Planner::valid_stamp(const builtin_interfaces::msg::Time &stamp) const
{
  const double delta = now().seconds()-rclcpp::Time(stamp).seconds();
  return std::isfinite(delta) && delta >= -0.15 && delta <= input_timeout_;
}
bool ExplorationNav2Planner::fresh(const Input &input) const
{
  if (!input.received || age(*input.received) > input_timeout_) return false;
  const double delta = now().seconds()-input.stamp;
  return delta >= -0.15 && delta <= input_timeout_;
}
bool ExplorationNav2Planner::ready() const
{
  return enabled_ && !complete_ && !shutting_down_ && fresh(target_input_) && fresh(odom_input_);
}
void ExplorationNav2Planner::on_target(const geometry_msgs::msg::PointStamped &message)
{
  if (!enabled_ || complete_ || shutting_down_ || message.header.frame_id != input_frame_ ||
      !valid_stamp(message.header.stamp) || !finite(message.point)) return;
  target_ = message.point;
  target_input_ = {rclcpp::Time(message.header.stamp).seconds(), Clock::now()};
}
void ExplorationNav2Planner::on_odometry(const nav_msgs::msg::Odometry &message)
{
  const auto &q = message.pose.pose.orientation;
  const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (message.header.frame_id != input_frame_ || !valid_stamp(message.header.stamp) ||
      !finite(message.pose.pose.position) || !std::isfinite(norm) || std::abs(norm-1.0) > 0.1) return;
  position_ = message.pose.pose.position;
  odom_input_ = {rclcpp::Time(message.header.stamp).seconds(), Clock::now()};
}
void ExplorationNav2Planner::tick()
{
  if (request_)
  {
    if (!ready() || age(request_->started) > planning_timeout_)
    {
      if (age(request_->started) > planning_timeout_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Nav2 planning timed out; canceling and waiting for terminal reply");
      cancel();
    }
    return; // One request at a time, including delayed acceptance/cancellation.
  }
  if (!ready()) return;
  if (!client_->action_server_is_ready())
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for Nav2 /compute_path_through_poses");
    return;
  }
  if (retry_after_ && Clock::now() < *retry_after_) return;
  const bool target_changed = !last_sent_ || distance(target_, last_requested_) >= target_update_distance_;
  if (!target_changed && age(*last_sent_) < replan_interval_) return;
  send();
}
void ExplorationNav2Planner::send()
{
  try
  {
    const auto tf = tf_->lookupTransform(planning_frame_, input_frame_, tf2::TimePointZero);
    geometry_msgs::msg::PointStamped in, out;
    in.point = target_;
    tf2::doTransform(in, out, tf);
    Plan::Goal goal;
    goal.planner_id = planner_id_;
    goal.use_start = false; // Nav2 obtains the current robot pose from its costmap/TF.
    geometry_msgs::msg::PoseStamped endpoint;
    endpoint.header.frame_id = planning_frame_;
    endpoint.header.stamp = now();
    endpoint.pose.position = out.point;
    in.point = position_;
    geometry_msgs::msg::PointStamped robot;
    tf2::doTransform(in, robot, tf);
    const double yaw = std::atan2(out.point.y-robot.point.y, out.point.x-robot.point.x);
    endpoint.pose.orientation.z = std::sin(yaw/2);
    endpoint.pose.orientation.w = std::cos(yaw/2);
    if(corridor_enabled_&&valid_stamp(centerline_.header.stamp)&&centerline_.poses.size()>=2) {
      std::vector<CorridorSection> line;
      for(const auto&p:centerline_.poses)if(finite(p.pose.position))line.push_back({p.pose.position.x,p.pose.position.y,1.});
      if(line.size()>=2) {
        const auto projection=project_corridor(line,target_.x,target_.y);
        geometry_msgs::msg::PoseStamped tangent,transformed;tangent.header=centerline_.header;
        tangent.pose.orientation.z=std::sin(projection.yaw/2);tangent.pose.orientation.w=std::cos(projection.yaw/2);
        tf2::doTransform(tangent,transformed,tf);endpoint.pose.orientation=transformed.pose.orientation;
      }
    }
    goal.goals.push_back(endpoint);
    const auto id = ++next_request_;
    request_ = Request{id, target_, Clock::now(), nullptr, false, false};
    last_requested_ = target_;
    last_sent_ = Clock::now();
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.goal_response_callback = [this, id](Handle::SharedPtr handle) {
      if (!request_ || request_->id != id) return;
      if (!handle)
      {
        RCLCPP_WARN(get_logger(), "Nav2 planning request %lu rejected; waiting and retrying",
            static_cast<unsigned long>(id));
        request_.reset();
        retry_after_ = Clock::now()+std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(retry_delay_));
        return;
      }
      request_->handle = handle;
      if (request_->discard || !ready()) cancel();
    };
    options.result_callback = [this, id](const Handle::WrappedResult &result) { finish(id, result); };
    (void)client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "Nav2 planning request %lu: goal=(%.2f, %.2f) frame=%s",
        static_cast<unsigned long>(id), endpoint.pose.position.x, endpoint.pose.position.y,
        planning_frame_.c_str());
  }
  catch (const std::exception &error)
  {
    request_.reset();
    retry_after_ = Clock::now()+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(retry_delay_));
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Cannot request Nav2 path: %s", error.what());
  }
}
void ExplorationNav2Planner::finish(uint64_t id, const Handle::WrappedResult &result)
{
  if (!request_ || request_->id != id) return;
  const auto request = *request_;
  request_.reset();
  if (request.discard || !ready() || age(request.started) > planning_timeout_ ||
      distance(request.target, target_) >= target_update_distance_)
  {
    RCLCPP_INFO(get_logger(), "Discarded superseded/stale planning result %lu", static_cast<unsigned long>(id));
    return;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result)
  {
    retry_after_ = Clock::now()+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(retry_delay_));
    RCLCPP_WARN(get_logger(), "Nav2 planner failed for request %lu (code=%d); no TARE-path fallback",
        static_cast<unsigned long>(id), static_cast<int>(result.code));
    return;
  }
  publish(result.result->path, request.target);
}
void ExplorationNav2Planner::publish(const nav_msgs::msg::Path &path,
                                    const geometry_msgs::msg::Point &requested)
{
  try
  {
    if (path.header.frame_id != planning_frame_ || path.poses.size() < 2 || path.poses.size() > 20000)
      throw std::runtime_error("invalid path frame or size");
    const auto tf = tf_->lookupTransform(input_frame_, planning_frame_, tf2::TimePointZero);
    nav_msgs::msg::Path transformed;
    transformed.header.frame_id = input_frame_;
    transformed.header.stamp = now();
    for (const auto &pose : path.poses)
    {
      if (!finite(pose.pose.position) ||
          (!pose.header.frame_id.empty() && pose.header.frame_id != planning_frame_))
        throw std::runtime_error("nonfinite path or inconsistent pose frame");
      geometry_msgs::msg::PoseStamped output;
      tf2::doTransform(pose, output, tf);
      output.header = transformed.header;
      if (transformed.poses.empty() || distance(transformed.poses.back().pose.position, output.pose.position)>1e-5)
        transformed.poses.push_back(output);
    }
    if (transformed.poses.size()<2 ||
        distance(transformed.poses.front().pose.position, position_)>start_tolerance_ ||
        distance(transformed.poses.back().pose.position, requested)>endpoint_tolerance_)
      throw std::runtime_error("path endpoints outside tolerance or no usable segment");
    path_pub_->publish(transformed);
    retry_after_.reset();
    RCLCPP_INFO(get_logger(), "Nav2 path ready: %zu poses, target=(%.2f, %.2f)",
        transformed.poses.size(), requested.x, requested.y);
  }
  catch (const std::exception &error)
  {
    retry_after_ = Clock::now()+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(retry_delay_));
    nav_msgs::msg::Path withdrawn;withdrawn.header.frame_id=input_frame_;withdrawn.header.stamp=now();
    path_pub_->publish(withdrawn);
    RCLCPP_WARN(get_logger(), "Rejected invalid Nav2 path: %s", error.what());
  }
}
void ExplorationNav2Planner::cancel()
{
  if (!request_) return;
  request_->discard = true;
  if (!request_->handle || request_->cancel_sent) return;
  try
  {
    (void)client_->async_cancel_goal(request_->handle);
    request_->cancel_sent = true;
  }
  catch (const std::exception &error)
  {
    RCLCPP_WARN(get_logger(), "Planner cancellation failed: %s", error.what());
  }
}
void ExplorationNav2Planner::stop()
{
  shutting_down_ = true;
  cancel();
}
} // namespace rover_explore
