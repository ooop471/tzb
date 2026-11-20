// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#include "rover_explore/exploration_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rover_explore
{
namespace
{
double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
bool finite(const geometry_msgs::msg::Point &p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
double elapsed(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}
geometry_msgs::msg::Point projection(const geometry_msgs::msg::Point &p,
                                   const geometry_msgs::msg::Point &a,
                                   const geometry_msgs::msg::Point &b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double squared = dx * dx + dy * dy;
  const double t = squared > 1e-12 ?
      std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / squared, 0.0, 1.0) : 0.0;
  geometry_msgs::msg::Point q;
  q.x = a.x + t * dx;
  q.y = a.y + t * dy;
  q.z = a.z + t * (b.z - a.z);
  return q;
}
} // namespace

ExplorationPathFollower::ExplorationPathFollower(const rclcpp::NodeOptions &options)
    : Node("exploration_path_follower", options)
{
  input_timeout_ = declare_parameter("input_timeout", 2.0);
  route_timeout_ = declare_parameter("route_timeout", 8.0);
  update_interval_ = declare_parameter("update_interval", 0.5);
  update_distance_ = declare_parameter("update_distance", 0.15);
  handoff_distance_ = declare_parameter("handoff_distance", 0.8);
  start_tolerance_ = declare_parameter("start_tolerance", 0.8);
  goal_tolerance_ = declare_parameter("goal_tolerance", 0.25);
  retry_delay_ = declare_parameter("retry_delay", 1.0);
  cancel_timeout_ = declare_parameter("cancel_timeout", 5.0);
  retry_limit_ = declare_parameter("retry_limit", 3);
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom_init");
  tracking_frame_ = declare_parameter<std::string>("tracking_frame", "map");
  controller_ = declare_parameter<std::string>("controller_id", "FollowPath");
  goal_checker_ = declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
  managed_ = declare_parameter("managed_mode", false);
  lease_timeout_ = declare_parameter("management_lease_timeout", 2.0);
  if (!std::isfinite(lease_timeout_) || lease_timeout_ <= 0)
    throw std::invalid_argument("management_lease_timeout must be positive");
  for (double v : {input_timeout_, route_timeout_, update_interval_, update_distance_,
                   handoff_distance_, start_tolerance_, goal_tolerance_, retry_delay_, cancel_timeout_})
  {
    if (!std::isfinite(v) || v <= 0.0)
    {
      throw std::invalid_argument("Follower timeouts and distances must be finite and positive");
    }
  }
  if (retry_limit_ < 0 || odom_frame_.empty() || tracking_frame_.empty())
  {
    throw std::invalid_argument("retry_limit must be >= 0; frames must not be empty");
  }
  tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, this, false);
  client_ = rclcpp_action::create_client<Follow>(this, declare_parameter<std::string>("follow_action", "/follow_path"));
  const auto latched = rclcpp::QoS(1).transient_local();
  enable_pub_ = create_publisher<std_msgs::msg::Bool>("/explore/enabled", latched);
  submitted_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/executed_path", latched);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odometry(std::move(m)); });
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_cloud(*m, cloud_input_); });
  terrain_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/terrain_map", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_cloud(*m, terrain_input_); });
  route_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/explore/control_path", 2,
      [this](nav_msgs::msg::Path::ConstSharedPtr m) { on_route(std::move(m)); });
  planner_sub_ = create_subscription<std_msgs::msg::String>(
      "/explore/status", latched, [this](std_msgs::msg::String::ConstSharedPtr m)
      {
        if (enabled_ && m->data == "EXITED") exit_complete_ = true;
        if (enabled_ && m->data == "EXIT_FORWARD" && !exit_forward_) {
          exit_forward_ = true;
          failures_=0;failed_route_.clear();retry_after_.reset();
        }
        planner_status_ = m->data;
        planner_input_.received = Clock::now();
        planner_input_.stamp = now().seconds();
      });
  coverage_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/explore/coverage_complete", 2,
      [this](std_msgs::msg::Bool::ConstSharedPtr m)
      {
        if (m->data != complete_)
        {
          // Never replay the route from before completion; wait for a new plan.
          route_.clear();
          route_input_ = {};
          failed_route_.clear();
          failures_ = 0;
        }
        complete_ = m->data;
      });
  // Publish the initial disabled value; tick enables planning once sensors/Nav2/TF are ready.
  std_msgs::msg::Bool disabled;
  enable_pub_->publish(disabled);
  management_pub_ = create_publisher<std_msgs::msg::String>("/exploration/state", 10);
  if (managed_)
  {
    management_sub_ = create_subscription<std_msgs::msg::String>(
      "/exploration/command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) { management_command(m->data); });
  }
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
    tick();
    management_status();
  });
  RCLCPP_INFO(get_logger(), "Exploration follower: continuous FollowPath "
              "updates and replanning; automatic retreat/turn recovery removed");
}

void ExplorationPathFollower::management_command(const std::string &data)
{
  try
  {
    const auto j = nlohmann::json::parse(data);
    const auto session = j.at("session").get<std::string>();
    const auto generation = j.at("generation").get<uint64_t>();
    const bool requested = j.at("enabled").get<bool>();
    const double age = now().seconds() - j.at("stamp").get<double>();
    if (session.empty() || !std::isfinite(age) || age < -0.15 || age > lease_timeout_) return;
    // Ownership changes must first arrive as a disable command.
    if (session != management_session_ && requested) return;
    if (session == management_session_ && generation < management_generation_) return;
    if (session != management_session_ || generation != management_generation_ || requested != requested_)
    {
      enable(false);
      route_.clear(); submitted_.clear(); failed_route_.clear();
      route_input_ = {}; planner_input_ = {}; failures_ = 0;
      terminal_.reset(); retry_after_.reset();
      resume_stamp_ = now().seconds();
      if (!requested || !requests_.empty()) cancel();
    }
    const bool fresh_generation=session!=management_session_ || generation!=management_generation_;
    management_session_ = session;
    management_generation_ = generation;
    if (requested && (fresh_generation || !requested_))
      {exit_complete_ = false; exit_forward_ = false;}
    requested_ = requested;
    management_received_ = Clock::now();
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid exploration command: %s", e.what());
  }
}

void ExplorationPathFollower::management_status()
{
  std_msgs::msg::String m;
  const bool lease = management_received_ && elapsed(*management_received_) <= lease_timeout_;
  m.data = nlohmann::json({{"session", management_session_}, {"generation", management_generation_},
    {"stamp", now().seconds()}, {"enabled", enabled_}, {"requested", managed_ ? requested_ && lease : enabled_},
    {"idle", !enabled_ && requests_.empty()}, {"pending", requests_.size()},
    {"recovering",false}, {"contact_adjustment",false}, {"reverse_step_complete",false}, {"recovery_kind","NONE"}, {"recovery_attempts",0},
    {"coverage_complete", complete_}, {"exit_complete", exit_complete_}, {"exit_forward", exit_forward_},
    {"route_ready", enabled_ && fresh(route_input_, route_timeout_) && !route_.empty()}}).dump();
  management_pub_->publish(m);
}

bool ExplorationPathFollower::valid_stamp(const builtin_interfaces::msg::Time &stamp) const
{
  const double age = now().seconds() - rclcpp::Time(stamp).seconds();
  return std::isfinite(age) && age >= -0.15 && age <= input_timeout_;
}
bool ExplorationPathFollower::fresh(const Input &input, double timeout) const
{
  if (!input.received || elapsed(*input.received) > timeout)
  {
    return false;
  }
  const double age = now().seconds() - input.stamp;
  return age >= -0.15 && age <= timeout;
}
void ExplorationPathFollower::on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr m)
{
  const auto &q = m->pose.pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (m->header.frame_id != odom_frame_ || !valid_stamp(m->header.stamp) ||
      !finite(m->pose.pose.position) || !std::isfinite(norm) || std::abs(norm - 1.0) > 0.1)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignored invalid/stale odometry or frame");
    return;
  }
  position_ = m->pose.pose.position;
  odom_input_ = {rclcpp::Time(m->header.stamp).seconds(), Clock::now()};
}
void ExplorationPathFollower::on_cloud(const sensor_msgs::msg::PointCloud2 &m, Input &input)
{
  if (m.header.frame_id == odom_frame_ && valid_stamp(m.header.stamp))
  {
    input = {rclcpp::Time(m.header.stamp).seconds(), Clock::now()};
  }
}
void ExplorationPathFollower::on_route(nav_msgs::msg::Path::ConstSharedPtr m)
{
  if (!enabled_ || complete_ || exit_complete_ || canceling_ || m->header.frame_id != odom_frame_ ||
      !valid_stamp(m->header.stamp) ||
      (managed_ && rclcpp::Time(m->header.stamp).seconds() <= resume_stamp_))
  {
    return;
  }
  if(m->poses.empty()) {
    route_.clear();route_input_={};cancel();
    explain("Planner withdrew route; waiting for a corridor-feasible path");
    return;
  }
  if(m->poses.size()<2)return;
  Points points;
  for (const auto &pose : m->poses)
  {
    const auto &p = pose.pose.position;
    if (!finite(p))
    {
      RCLCPP_WARN(get_logger(), "Ignored route with nonfinite coordinates");
      return;
    }
    if (points.empty() || distance(points.back(), p) > 1e-5)
    {
      points.push_back(p);
    }
  }
  if (points.size() >= 2)
  {
    route_ = std::move(points);
    route_input_ = {rclcpp::Time(m->header.stamp).seconds(), Clock::now()};
  }
}
std::string ExplorationPathFollower::inputs_problem() const
{
  if (!fresh(odom_input_, input_timeout_)) { return "waiting for fresh /odometry"; }
  if (!fresh(cloud_input_, input_timeout_)) { return "waiting for fresh /cloud_registered"; }
  if (!fresh(terrain_input_, input_timeout_)) { return "waiting for fresh /terrain_map"; }
  if (!client_->action_server_is_ready()) { return "waiting for Nav2 /follow_path"; }
  if (!tf_->canTransform(tracking_frame_, odom_frame_, tf2::TimePointZero))
  {
    return "waiting for " + tracking_frame_ + " <- " + odom_frame_ + " TF";
  }
  return {};
}
void ExplorationPathFollower::enable(bool value)
{
  if (enabled_ != value)
  {
    enabled_ = value;
    std_msgs::msg::Bool m;
    m.data = value;
    enable_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "Exploration %s", value ? "enabled automatically" : "disabled");
  }
}
void ExplorationPathFollower::explain(const std::string &reason)
{
  if (last_reason_ != reason)
  {
    RCLCPP_INFO(get_logger(), "%s", reason.c_str());
    last_reason_ = reason;
  }
}

ExplorationPathFollower::Points ExplorationPathFollower::remaining(const Points &points) const
{
  if (points.size() < 2) { return {}; }
  std::size_t segment = 0;
  double best = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point projected;
  for (std::size_t i = 0; i + 1 < points.size(); ++i)
  {
    const auto p = projection(position_, points[i], points[i + 1]);
    const double d = distance(position_, p);
    if (d < best)
    {
      best = d;
      segment = i;
      projected = p;
    }
  }
  if (best > start_tolerance_) { return {}; }
  Points result{projected};
  for (std::size_t i = segment + 1; i < points.size(); ++i)
  {
    if (distance(result.back(), points[i]) > 1e-5) { result.push_back(points[i]); }
  }
  return result;
}

bool ExplorationPathFollower::different(const Points &a, const Points &b) const
{
  if (a.size() < 2 || b.size() < 2) { return true; }
  if (distance(a.back(), b.back()) >= update_distance_) { return true; }
  const auto length = [](const Points &points)
  {
    double total = 0;
    for (std::size_t i = 1; i < points.size(); ++i)
    {
      total += distance(points[i - 1], points[i]);
    }
    return total;
  };
  const double la = length(a), lb = length(b);
  if (std::abs(la - lb) >= update_distance_) { return true; }
  const auto at = [](const Points &points, double arc)
  {
    for (std::size_t i = 1; i < points.size(); ++i)
    {
      const double segment = distance(points[i - 1], points[i]);
      if (arc <= segment && segment > 1e-9)
      {
        auto p = points[i - 1];
        const double ratio = arc / segment;
        p.x += ratio * (points[i].x - p.x);
        p.y += ratio * (points[i].y - p.y);
        return p;
      }
      arc -= segment;
    }
    return points.back();
  };
  // Compare ordered arc samples, not just the endpoint or the unordered set of vertices.
  for (double arc = 0; arc < std::max(la, lb); arc += 0.1)
  {
    if (distance(at(a, arc), at(b, arc)) >= update_distance_) { return true; }
  }
  return false;
}

std::optional<nav_msgs::msg::Path> ExplorationPathFollower::transform_path(const Points &points)
{
  try
  {
    const auto tf = tf_->lookupTransform(tracking_frame_, odom_frame_, tf2::TimePointZero);
    nav_msgs::msg::Path path;
    path.header.frame_id = tracking_frame_;
    path.header.stamp = now();
    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
      const auto &a = points[i];
      const auto &b = points[i + 1];
      const double steps = std::ceil(distance(a, b) / 0.15);
      if (!std::isfinite(steps) || steps > 20000 || path.poses.size() + steps > 20000)
      {
        explain("Ignored implausibly large route during interpolation");
        return std::nullopt;
      }
      const int count = std::max(1, static_cast<int>(steps));
      for (int j = 0; j < count; ++j)
      {
        geometry_msgs::msg::PointStamped p, transformed;
        p.header.frame_id = odom_frame_;
        p.point.x = a.x + (b.x - a.x) * j / count;
        p.point.y = a.y + (b.y - a.y) * j / count;
        p.point.z = a.z + (b.z - a.z) * j / count;
        tf2::doTransform(p, transformed, tf);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = transformed.point;
        path.poses.push_back(pose);
      }
    }
    geometry_msgs::msg::PointStamped end, transformed;
    end.point = points.back();
    tf2::doTransform(end, transformed, tf);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = transformed.point;
    path.poses.push_back(pose);
    for (std::size_t i = 0; i < path.poses.size(); ++i)
    {
      const auto &a = path.poses[i > 0 ? i - 1 : 0].pose.position;
      const auto &b = path.poses[std::min(i + 1, path.poses.size() - 1)].pose.position;
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);
      path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
      path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
    }
    return path;
  }
  catch (const tf2::TransformException &e)
  {
    explain(std::string("Cannot transform route: ") + e.what());
    return std::nullopt;
  }
}

void ExplorationPathFollower::send(const nav_msgs::msg::Path &path, const Points &points)
{
  const bool update = !requests_.empty();
  const auto id = ++next_request_;
  requests_.emplace(id, Request{});
  Follow::Goal goal;
  goal.path = path;
  goal.controller_id = controller_;
  goal.goal_checker_id = goal_checker_;
  rclcpp_action::Client<Follow>::SendGoalOptions options;
  options.goal_response_callback = [this, id, points, path](Handle::SharedPtr handle)
  {
    const auto it = requests_.find(id);
    if (it == requests_.end()) { return; }
    if (!handle)
    {
      finish(id, rclcpp_action::ResultCode::ABORTED, "Nav2 rejected path");
      return;
    }
    it->second.handle = handle;
    submitted_ = points;
    submitted_pub_->publish(path);
    // An accepted replacement supersedes the predecessor; wait for every terminal reply.
    for (auto &[other, request] : requests_)
    {
      if (other != id) { request.replaced = true; }
    }
    terminal_.reset();
    if (canceling_ || shutting_down_) { cancel(); }
    RCLCPP_INFO(get_logger(), "FollowPath request %lu accepted", static_cast<unsigned long>(id));
  };
  options.result_callback = [this, id](const Handle::WrappedResult &r)
  {
    finish(id, r.code, "Nav2 terminal result");
  };
  options.feedback_callback = [this, id](Handle::SharedPtr, const std::shared_ptr<const Follow::Feedback> f)
  {
    const auto it = requests_.find(id);
    if (it != requests_.end() && !it->second.replaced)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "FollowPath request %lu: commanded linear speed=%.3f m/s, remaining=%.2f m",
          static_cast<unsigned long>(id), f->speed, f->distance_to_goal);
    }
  };
  try
  {
    (void)client_->async_send_goal(goal, options);
    sent_ = Clock::now();
    RCLCPP_INFO(get_logger(), "Route %s without cancel: request=%lu points=%zu "
        "end=(%.2f, %.2f); retry=%d/%d", update ? "updated" : "started",
        static_cast<unsigned long>(id), path.poses.size(), points.back().x, points.back().y,
        failures_, retry_limit_);
  }
  catch (const std::exception &e)
  {
    finish(id, rclcpp_action::ResultCode::ABORTED, e.what());
  }
}

void ExplorationPathFollower::finish(uint64_t id, rclcpp_action::ResultCode code,
                                     const std::string &reason)
{
  const auto it = requests_.find(id);
  if (it == requests_.end()) { return; }
  const bool replaced = it->second.replaced;
  const bool rejected = !it->second.handle;
  const bool rejected_update = !it->second.handle && (requests_.size() > 1 || terminal_);
  requests_.erase(it);
  RCLCPP_INFO(get_logger(), "FollowPath request %lu finished: code=%d replaced=%s %s",
      static_cast<unsigned long>(id), static_cast<int>(code), replaced ? "true" : "false",
      reason.c_str());
  if (!replaced && !rejected_update)
  {
    terminal_ = code;
    terminal_rejected_ = rejected;
  }
  if (rejected_update)
  {
    // Keep the predecessor active and rate-limit repeated update rejections.
    retry_after_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(retry_delay_));
    explain("Path update rejected; previous FollowPath remains active");
  }
  drain();
}
void ExplorationPathFollower::drain()
{
  if (!requests_.empty()) { return; }
  if (canceling_)
  {
    canceling_ = false;
    cancel_started_.reset();
    terminal_.reset();
    return;
  }
  if (!terminal_) { return; }
  if (terminal_rejected_)
  {
    // An action endpoint can be discovered before the lifecycle controller is active.
    // Rejected, unexecuted goals are readiness failures, not exhausted collision retries.
    retry_after_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(retry_delay_));
    explain("Nav2 not accepting routes yet; waiting and retrying without a movement command");
  }
  else if (*terminal_ == rclcpp_action::ResultCode::SUCCEEDED)
  {
    failures_ = 0;
    failed_route_.clear();
    explain("FollowPath completed; waiting for the next TARE route");
  }
  else
  {
    ++failures_;
    failed_route_ = submitted_;
    retry_after_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(retry_delay_));
    RCLCPP_WARN(get_logger(), "Controller failed; retry %d/%d after %.1f s using a fresh route. "
        "No endpoint blacklist; see controller_server log for the underlying error",
        failures_, retry_limit_, retry_delay_);
  }
  terminal_.reset();
}
void ExplorationPathFollower::cancel()
{
  if (requests_.empty()) { return; }
  if (!canceling_)
  {
    canceling_ = true;
    cancel_started_ = Clock::now();
  }
  for (auto &[id, request] : requests_)
  {
    (void)id;
    if (!request.handle || request.cancel_sent) { continue; }
    try
    {
      (void)client_->async_cancel_goal(request.handle);
      request.cancel_sent = true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Cancel request failed: %s", e.what());
    }
  }
}

void ExplorationPathFollower::tick()
{
  if (shutting_down_) { cancel(); return; }
  if (managed_ && (!requested_ || !management_received_ || elapsed(*management_received_) > lease_timeout_))
  {
    enable(false);
    route_.clear(); route_input_ = {};
    cancel();
    return;
  }
  if (exit_complete_)
  {
    enable(false); route_.clear(); route_input_ = {}; cancel();
    explain("Cloud exit confirmed and forward leg completed; waiting for mission handoff");
    return;
  }
  if (canceling_)
  {
    if (cancel_started_ && elapsed(*cancel_started_) > cancel_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "Still waiting for Nav2 terminal cancellation; new goals remain blocked");
    }
    return;
  }
  const auto problem = inputs_problem();
  if (!problem.empty())
  {
    enable(false);
    route_.clear();
    explain(problem);
    cancel();
    return;
  }
  // Keep the planner enabled: new geometry must be able to invalidate completion.
  enable(true);
  if (complete_)
  {
    cancel();
    explain("TARE coverage complete; monitoring observations for new exploration targets");
    return;
  }
  if (!fresh(planner_input_, route_timeout_) || planner_status_ == "WAITING_FOR_INPUT")
  {
    explain("Waiting for planner data: " + planner_status_);
    cancel();
    return;
  }
  if (!fresh(route_input_, route_timeout_))
  {
    explain("No fresh /explore/control_path; planner=" + planner_status_);
    cancel();
    return;
  }
  const auto points = remaining(route_);
  if (points.size() < 2)
  {
    explain("Route start outside tolerance or no remaining segment; waiting for a usable route");
    return;
  }
  if (distance(points.back(), position_) < goal_tolerance_)
  {
    explain("Near route endpoint; waiting for TARE to extend the route");
    return;
  }
  if (retry_after_ && Clock::now() < *retry_after_) { return; }
  if (!failed_route_.empty() && different(remaining(failed_route_), points))
  {
    failures_ = 0;
    failed_route_.clear();
  }
  if (requests_.empty() && failures_ > retry_limit_)
  {
    // Continue retrying fresh paths without retreat/spin commands. Never declare
    // completion or permanently blacklist geometry merely because tracking failed.
    failures_=0;
    retry_after_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(retry_delay_));
    return;
  }
  if (!requests_.empty())
  {
    // At most one replacement in flight. Never cancel an action merely to update its path.
    if (requests_.size() != 1 || !requests_.begin()->second.handle ||
        !different(remaining(submitted_), points)) { return; }
    const bool near_end = !submitted_.empty() &&
        distance(position_, submitted_.back()) < handoff_distance_;
    if (!near_end && sent_ && elapsed(*sent_) < update_interval_) { return; }
  }
  const auto path = transform_path(points);
  if (path)
  {
    explain("Following TARE route");
    send(*path, points);
  }
}
void ExplorationPathFollower::stop()
{
  shutting_down_ = true;
  enable(false);
  cancel();
}
} // namespace rover_explore
