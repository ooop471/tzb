// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef BRINGUP__EXPLORATION_PATH_FOLLOWER_HPP_
#define BRINGUP__EXPLORATION_PATH_FOLLOWER_HPP_

#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rover_explore
{
// Follow planned exploration paths; wait and replan without automatic retreat/spin.
class ExplorationPathFollower : public rclcpp::Node
{
public:
  explicit ExplorationPathFollower(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void stop();
  bool stopped() const { return requests_.empty(); }

private:
  friend struct ExplorationFollowerTest;
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Follow = nav2_msgs::action::FollowPath;
  using Handle = rclcpp_action::ClientGoalHandle<Follow>;
  using Points = std::vector<geometry_msgs::msg::Point>;
  struct Request
  {
    Handle::SharedPtr handle;
    bool replaced{false};
    bool cancel_sent{false};
  };
  struct Input
  {
    double stamp{0};
    std::optional<Time> received;
  };

  void tick();
  void on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void on_route(nav_msgs::msg::Path::ConstSharedPtr message);
  void on_cloud(const sensor_msgs::msg::PointCloud2 &message, Input &input);
  bool fresh(const Input &input, double timeout) const;
  bool valid_stamp(const builtin_interfaces::msg::Time &stamp) const;
  std::string inputs_problem() const;
  void enable(bool value);
  void explain(const std::string &reason);
  void cancel();
  void send(const nav_msgs::msg::Path &path, const Points &points);
  void finish(uint64_t id, rclcpp_action::ResultCode code, const std::string &reason);
  void drain();
  void management_command(const std::string &json);
  void management_status();
  Points remaining(const Points &points) const;
  bool different(const Points &a, const Points &b) const;
  std::optional<nav_msgs::msg::Path> transform_path(const Points &points);

  double input_timeout_, route_timeout_, update_interval_, update_distance_;
  double handoff_distance_, start_tolerance_, goal_tolerance_, retry_delay_, cancel_timeout_;
  int retry_limit_;
  std::string odom_frame_, tracking_frame_, controller_, goal_checker_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp_action::Client<Follow>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr submitted_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_, terrain_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coverage_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr management_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr management_pub_;
  bool managed_{false}, requested_{false}, exit_complete_{false}, exit_forward_{false};
  double lease_timeout_{0.6}, resume_stamp_{0.0};
  std::string management_session_;
  uint64_t management_generation_{0};
  std::optional<Time> management_received_;
  Input odom_input_, cloud_input_, terrain_input_, route_input_, planner_input_;
  geometry_msgs::msg::Point position_;
  Points route_, submitted_, failed_route_;
  std::string planner_status_, last_reason_;
  std::map<uint64_t, Request> requests_;
  std::optional<rclcpp_action::ResultCode> terminal_;
  std::optional<Time> sent_, retry_after_, cancel_started_;
  uint64_t next_request_{0};
  int failures_{0};
  bool terminal_rejected_{false};
  bool enabled_{false}, canceling_{false}, shutting_down_{false}, complete_{false};
};
} // namespace rover_explore
#endif  // BRINGUP__EXPLORATION_PATH_FOLLOWER_HPP_
