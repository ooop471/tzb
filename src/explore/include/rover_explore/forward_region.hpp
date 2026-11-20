// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace rover_explore
{
// Restricts exploration goals, while the navigation graph keeps its full extent.
struct ForwardExplorationRegion
{
  bool enabled = true;
  double forward_distance = 4.0;
  double half_width = 4.0;
  double entry_margin = 0.05;

  void validate() const
  {
    if (!std::isfinite(forward_distance) || forward_distance <= 0 ||
        !std::isfinite(half_width) || half_width <= 0 ||
        !std::isfinite(entry_margin) || entry_margin < 0 ||
        2 * entry_margin >= forward_distance || entry_margin >= half_width)
      throw std::invalid_argument("exploration_region requires positive extents and a smaller nonnegative entry_margin");
  }

  bool contains(const Eigen::Vector3d &point, const Eigen::Vector3d &robot,
                double yaw, bool previously_allowed = false) const
  {
    if (!point.allFinite() || !robot.allFinite() || !std::isfinite(yaw)) return false;
    if (!enabled) return true;
    const auto delta = point - robot;
    const double x = std::cos(yaw) * delta.x() + std::sin(yaw) * delta.y();
    const double y = -std::sin(yaw) * delta.x() + std::cos(yaw) * delta.y();
    // Entry/exit hysteresis stays entirely within the front rectangle.
    const double margin = previously_allowed ? 0.0 : entry_margin;
    return x >= margin && x <= forward_distance - margin &&
           std::abs(y) <= half_width - margin;
  }
};
}  // namespace rover_explore
