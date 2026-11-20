// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef ROVER_EXPLORE__CONFIGURATION_HPP_
#define ROVER_EXPLORE__CONFIGURATION_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include "rover_explore/goal_selection.hpp"
#include "rover_explore/corridor_centerline.hpp"

namespace rover_explore
{
/// Matches the existing navigation terrain node's horizontal radius and box.
struct SelfFilterBox
{
  bool enabled{true};
  double radius{0.6};
  Eigen::Vector3d minimum{0.20, -0.12, 0.05};
  Eigen::Vector3d maximum{0.36, 0.08, 0.60};
  void validate() const;
  bool contains_world(const Eigen::Vector3d &point, const Eigen::Vector3d &position,
                      const Eigen::Matrix3d &body_rotation) const;
};

SelfFilterBox read_self_filter(const rclcpp::Node &node);
GoalSelectionParameters read_goal_selection(const rclcpp::Node &node);
CorridorParameters read_corridor(const rclcpp::Node &node);
/// Fail before allocating grids when dimensions, units or coupled ranges disagree.
void validate_configuration(const rclcpp::Node &node);
} // namespace rover_explore
#endif // ROVER_EXPLORE__CONFIGURATION_HPP_
