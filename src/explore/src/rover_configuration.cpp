// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#include "rover_explore/configuration.hpp"
#include "rover_explore/forward_region.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace rover_explore
{
namespace
{
void require(bool condition, const std::string &message)
{
  if (!condition)
  {
    throw std::invalid_argument(message);
  }
}
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
} // namespace

void SelfFilterBox::validate() const
{
  require(std::isfinite(radius) && radius >= 0.0,
          "self_filter radius must be finite and nonnegative");
  require(minimum.allFinite() && maximum.allFinite() && (minimum.array() < maximum.array()).all(),
          "self_filter bounds must be finite, with minimum < maximum on each axis");
}

bool SelfFilterBox::contains_world(const Eigen::Vector3d &point, const Eigen::Vector3d &position,
                                   const Eigen::Matrix3d &body_rotation) const
{
  if (!enabled)
  {
    return false;
  }
  // The verified terrain node uses yaw-aligned XY and gravity-aligned Z.
  const Eigen::Vector3d delta = point - position;
  const double yaw = std::atan2(body_rotation(1, 0), body_rotation(0, 0));
  const Eigen::Vector3d local(std::cos(yaw) * delta.x() + std::sin(yaw) * delta.y(),
                              -std::sin(yaw) * delta.x() + std::cos(yaw) * delta.y(), delta.z());
  if (local.head<2>().squaredNorm() <= radius * radius)
  {
    return true;
  }
  return (local.array() >= minimum.array()).all() && (local.array() <= maximum.array()).all();
}

SelfFilterBox read_self_filter(const rclcpp::Node &node)
{
  SelfFilterBox result;
  result.enabled = node.get_parameter("self_filter.enabled").as_bool();
  result.radius = node.get_parameter("self_filter.radius").as_double();
  const auto minimum = node.get_parameter("self_filter.minimum").as_double_array();
  const auto maximum = node.get_parameter("self_filter.maximum").as_double_array();
  require(minimum.size() == 3 && maximum.size() == 3, "self_filter bounds need three XYZ values");
  result.minimum = {minimum[0], minimum[1], minimum[2]};
  result.maximum = {maximum[0], maximum[1], maximum[2]};
  result.validate();
  return result;
}

GoalSelectionParameters read_goal_selection(const rclcpp::Node &node)
{
  GoalSelectionParameters result;
  result.enabled=node.get_parameter("goal_selection.enabled").as_bool();
  result.empty_cycles=node.get_parameter("goal_selection.empty_cycles").as_int();
  result.main_cone_degrees=node.get_parameter("goal_selection.main_cone_degrees").as_double();
  result.max_turn_degrees=node.get_parameter("goal_selection.max_turn_degrees").as_double();
  result.min_distance=node.get_parameter("goal_selection.min_distance").as_double();
  result.max_distance=node.get_parameter("goal_selection.max_distance").as_double();
  result.target_distance=node.get_parameter("goal_selection.target_distance").as_double();
  result.handoff_distance=node.get_parameter("goal_selection.handoff_distance").as_double();
  result.progress_distance=node.get_parameter("goal_selection.progress_distance").as_double();
  result.progress_timeout=node.get_parameter("goal_selection.progress_timeout").as_double();
  result.retry_cooldown=node.get_parameter("goal_selection.retry_cooldown").as_double();
  result.retry_radius=node.get_parameter("goal_selection.retry_radius").as_double();
  result.validate();
  return result;
}

CorridorParameters read_corridor(const rclcpp::Node &node)
{
  CorridorParameters p;
  p.enabled=node.get_parameter("corridor.enabled").as_bool();
  p.nominal_width=node.get_parameter("corridor.nominal_width").as_double();
  p.minimum_width=node.get_parameter("corridor.minimum_width").as_double();
  p.maximum_width=node.get_parameter("corridor.maximum_width").as_double();
  p.transition_width=node.get_parameter("corridor.transition_width").as_double();
  p.wall_reference_height=node.get_parameter("corridor.wall_reference_height").as_double();
  p.sensor_height=node.get_parameter("corridor.ground.sensor_height").as_double();
  p.section_step=node.get_parameter("corridor.section_step").as_double();
  p.section_half_length=node.get_parameter("corridor.section_half_length").as_double();
  p.horizon=node.get_parameter("corridor.horizon").as_double();
  p.entry_lookahead=node.get_parameter("corridor.entry_lookahead").as_double();
  p.target_min_distance=node.get_parameter("corridor.target_min_distance").as_double();
  p.target_max_distance=node.get_parameter("corridor.target_max_distance").as_double();
  p.lookahead=node.get_parameter("corridor.lookahead").as_double();
  p.center_band=node.get_parameter("corridor.center_band").as_double();
  p.body_half_width=node.get_parameter("corridor.body_half_width").as_double();
  p.side_margin=node.get_parameter("corridor.side_margin").as_double();
  p.handoff_distance=node.get_parameter("corridor.handoff_distance").as_double();
  p.hold_time=node.get_parameter("corridor.hold_time").as_double();
  p.center_smoothing_time=node.get_parameter("corridor.center_smoothing_time").as_double();
  p.center_max_update=node.get_parameter("corridor.center_max_update").as_double();
  p.center_jump_threshold=node.get_parameter("corridor.center_jump_threshold").as_double();
  p.center_confirmation_cycles=node.get_parameter("corridor.center_confirmation_cycles").as_int();
  p.max_heading_degrees=node.get_parameter("corridor.max_heading_degrees").as_double();
  p.progress_timeout=node.get_parameter("corridor.progress_timeout").as_double();
  p.confirmation_cycles=node.get_parameter("corridor.confirmation_cycles").as_int();
  p.exit_enabled=node.get_parameter("corridor.exit_enabled").as_bool();
  p.dead_end_enabled=node.get_parameter("corridor.dead_end.enabled").as_bool();
  p.dead_end_distance=node.get_parameter("corridor.dead_end.detection_distance").as_double();
  p.body_front=node.get_parameter("corridor.body_front").as_double();
  p.body_rear=node.get_parameter("corridor.body_rear").as_double();
  p.exit_margin=node.get_parameter("corridor.exit_margin").as_double();
  p.exit_forward_distance=node.get_parameter("corridor.exit_forward_distance").as_double();
  p.exit_extension=node.get_parameter("corridor.exit_extension").as_double();
  p.exit_lookahead=node.get_parameter("corridor.exit_lookahead").as_double();
  p.validate();
  return p;
}

void validate_configuration(const rclcpp::Node &node)
{
  read_goal_selection(node);
  read_corridor(node);
  ForwardExplorationRegion region;
  region.enabled = node.get_parameter("exploration_region.enabled").as_bool();
  region.forward_distance = node.get_parameter("exploration_region.forward_distance").as_double();
  region.half_width = node.get_parameter("exploration_region.half_width").as_double();
  region.entry_margin = node.get_parameter("exploration_region.entry_margin").as_double();
  region.validate();
  const auto number = [&node](const std::string &name)
  { return node.get_parameter(name).as_double(); };
  const auto count = [&node](const std::string &name) { return node.get_parameter(name).as_int(); };
  for (const auto *name : {"input_timeout",
                           "sync_tolerance",
                           "kSensorRange",
                           "kNeighborRange",
                           "kLookAheadDistance",
                           "kKeyposeCloudDwzFilterLeafSize",
                           "kSurfaceCloudDwzLeafSize",
                           "kCollisionCloudDwzLeafSize",
                           "kCoveragePointCloudResolution",
                           "kPointCloudCellSize",
                           "kPointCloudCellHeight",
                           "kGridWorldCellHeight",
                           "kViewPointCollisionMargin",
                           "kViewPointCollisionMarginZPlus",
                           "kViewPointCollisionMarginZMinus",
                           "kCollisionGridZScale",
                           "kCoverageOcclusionThr",
                           "kCoverageDilationRadius",
                           "viewpoint_manager/resolution_x",
                           "viewpoint_manager/resolution_y",
                           "kCollisionGridResolutionX",
                           "kCollisionGridResolutionY",
                           "keypose_graph/kAddEdgeCollisionCheckResolution",
                           "keypose_graph/kAddEdgeCollisionCheckRadius"})
  {
    require(positive(number(name)), std::string(name) + " must be finite and positive");
  }
  const double angle = number("kViewpointVerticalHalfFovDeg");
  require(std::isfinite(angle) && angle > 0.0 && angle < 89.0,
          "kViewpointVerticalHalfFovDeg must be in (0, 89) degrees");
  require(count("viewpoint_manager/number_z") == 1, "Ground rover requires one viewpoint Z layer");
  for (const auto *name : {"viewpoint_manager/number_x", "viewpoint_manager/number_y"})
  {
    require(count(name) >= 5 && count(name) % 5 == 0,
            std::string(name) + " must be a positive multiple of 5 for rolling/TARE subspaces");
  }
  const double horizon_x =
      count("viewpoint_manager/number_x") * number("viewpoint_manager/resolution_x");
  const double horizon_y =
      count("viewpoint_manager/number_y") * number("viewpoint_manager/resolution_y");
  require(std::abs(horizon_x - horizon_y) < 1e-6,
          "TARE global cells derive from X; this rover profile requires equal X/Y horizons");
  for (const auto *name : {"kPointCloudManagerNeighborCellNum", "kGridWorldNearbyGridNum"})
  {
    require(count(name) >= 3 && count(name) % 2 == 1, std::string(name) + " must be odd and >= 3");
  }
  for (const auto *name :
       {"kPointCloudRowNum", "kPointCloudColNum", "kPointCloudLevelNum", "kGridWorldXNum",
        "kGridWorldYNum", "kGridWorldZNum", "kKeyposeCloudStackNum", "kMaxCellPointNum",
        "kFrontierClusterMinSize", "kCollisionPointThr"})
  {
    require(count(name) > 0, std::string(name) + " must be positive");
  }
  const auto neighbors = count("kPointCloudManagerNeighborCellNum");
  for (const auto *name : {"kPointCloudRowNum", "kPointCloudColNum", "kPointCloudLevelNum"})
  {
    require(count(name) >= neighbors,
            std::string(name) + " must contain the neighbor cloud window");
  }
  const double cell = number("kPointCloudCellSize");
  const double half_window = (neighbors - 1) * cell / 2.0;
  require(
      half_window >= std::max(horizon_x, horizon_y) / 2.0 + number("kViewPointCollisionMargin"),
      "Point-cloud neighbor window must contain the local viewpoint horizon plus collision margin");
  require(number("kSensorRange") <= half_window,
          "kSensorRange exceeds the retained point-cloud window");
  require(number("kLookAheadDistance") < std::min(horizon_x, horizon_y) / 2.0,
          "kLookAheadDistance must fit within the local viewpoint horizon");
  const std::vector<std::pair<const char *, double>> axes{
      {"x", cell}, {"y", cell}, {"z", number("kPointCloudCellHeight")}};
  for (const auto &axis : axes)
  {
    const std::string name = std::string("rolling_occupancy_grid/resolution_") + axis.first;
    const double resolution = number(name);
    require(positive(resolution), name + " must be finite and positive");
    const double cells = axis.second / resolution;
    require(cells >= 1.0 && std::abs(cells - std::round(cells)) < 1e-6,
            name + " must divide the point-cloud cell size for rolling alignment");
  }
  require(!node.get_parameter("kUseTerrainHeight").as_bool(),
          "This rover profile receives obstacle-only /terrain_map; terrain height estimation must "
          "stay disabled");
  require(std::isfinite(number("kTerrainCollisionThreshold")) &&
              number("kTerrainCollisionThreshold") >= 0.0,
          "kTerrainCollisionThreshold must be a nonnegative height in metres");
  (void)read_self_filter(node);
}
} // namespace rover_explore
