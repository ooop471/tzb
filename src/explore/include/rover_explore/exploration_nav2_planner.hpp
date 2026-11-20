// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef BRINGUP__EXPLORATION_NAV2_PLANNER_HPP_
#define BRINGUP__EXPLORATION_NAV2_PLANNER_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rover_explore
{
// Corridor mode forwards validated center paths; other modes request Nav2 paths.
class ExplorationNav2Planner : public rclcpp::Node
{
public:
  explicit ExplorationNav2Planner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void stop();
  bool stopped() const { return !request_; }

private:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Plan = nav2_msgs::action::ComputePathThroughPoses;
  using Handle = rclcpp_action::ClientGoalHandle<Plan>;
  struct Input
  {
    double stamp{0};
    std::optional<Time> received;
  };
  struct Request
  {
    uint64_t id;
    geometry_msgs::msg::Point target;
    Time started;
    Handle::SharedPtr handle;
    bool discard{false}, cancel_sent{false};
  };
  void on_target(const geometry_msgs::msg::PointStamped &message);
  void on_odometry(const nav_msgs::msg::Odometry &message);
  void tick();
  bool fresh(const Input &input) const;
  bool valid_stamp(const builtin_interfaces::msg::Time &stamp) const;
  bool ready() const;
  void cancel();
  void send();
  void finish(uint64_t id, const Handle::WrappedResult &result);
  void publish(const nav_msgs::msg::Path &path, const geometry_msgs::msg::Point &requested);

  bool corridor_enabled_;
  nav_msgs::msg::Path centerline_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr centerline_sub_;
  double input_timeout_, replan_interval_, planning_timeout_, retry_delay_;
  double target_update_distance_, endpoint_tolerance_, start_tolerance_;
  std::string input_frame_, planning_frame_, planner_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp_action::Client<Plan>::SharedPtr client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_, complete_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Input target_input_, odom_input_;
  geometry_msgs::msg::Point target_, position_, last_requested_;
  std::optional<Request> request_;
  std::optional<Time> last_sent_, retry_after_;
  uint64_t next_request_{0};
  bool enabled_{false}, complete_{false}, shutting_down_{false};
};
} // namespace rover_explore
#endif
