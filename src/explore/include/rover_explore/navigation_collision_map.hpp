// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include "rover_explore/corridor_dead_end.hpp"

namespace rover_explore
{
// A cycle-consistent view of Nav2's already inflated maps. Do not inflate again.
// OccupancyGrid: 99 = inscribed, 100 = lethal, -1 = unknown.
// Raw Costmap: 253 = inscribed, 254 = lethal, 255 = unknown.
class NavigationCollisionMap
{
public:
  bool prepare(nav_msgs::msg::OccupancyGrid::ConstSharedPtr global,
               nav2_msgs::msg::Costmap::ConstSharedPtr local,
               const Eigen::Isometry3d &world_to_global,
               const Eigen::Isometry3d &world_to_local,
               double now, double timeout, double future_tolerance)
  {
    ready_ = false;
    global_.reset(); local_.reset();
    const auto fresh = [&](const auto &message) {
      const auto &s = message.header.stamp;
      const double age = now - (s.sec + s.nanosec * 1e-9);
      return !message.header.frame_id.empty() && std::isfinite(age) &&
             age >= -future_tolerance && age <= timeout;
    };
    if (!global || !local || !std::isfinite(now) || !std::isfinite(timeout) || timeout <= 0 ||
        !std::isfinite(future_tolerance) || future_tolerance < 0 ||
        !fresh(*global) || !fresh(*local)) return false;
    if (!global_geometry_.prepare(global->info.resolution, global->info.width, global->info.height,
                                 global->info.origin, world_to_global, global->data.size()) ||
        !local_geometry_.prepare(local->metadata.resolution, local->metadata.size_x,
                                local->metadata.size_y, local->metadata.origin,
                                world_to_local, local->data.size())) return false;
    global_ = std::move(global); local_ = std::move(local);
    step_ = .5 * std::min(global_geometry_.resolution, local_geometry_.resolution);
    ready_ = true;
    return true;
  }

  void invalidate() { ready_ = false; }
  bool ready() const { return ready_; }
  double sample_step() const { return step_; }
  PhysicalCell physical_cell(const Eigen::Vector3d &world) const {
    if(!ready_||!world.allFinite())return PhysicalCell::Unknown;
    const auto gi=global_geometry_.index(world),li=local_geometry_.index(world);
    // Confirmation must be covered by both fresh maps, including the local one.
    if(gi<0||li<0)return PhysicalCell::Unknown;
    const int g=global_->data[gi],l=local_->data[li];
    if(g<0||l==255)return PhysicalCell::Unknown;
    if(g>=100&&l>=254)return PhysicalCell::Obstacle;
    if(g<99&&l<253)return PhysicalCell::Free;
    return PhysicalCell::Unknown;
  }
  bool traversable(const Eigen::Vector3d &world) const
  {
    if (!ready_ || !world.allFinite()) return false;
    const auto gi = global_geometry_.index(world);
    if (gi < 0) return false;
    const int g = global_->data[gi];
    if (g < 0 || g >= 99) return false;
    // The local rolling window is smaller than the exploration horizon. Inside
    // it, both planner and controller maps must agree; outside, use the global map.
    const auto li = local_geometry_.index(world);
    return li < 0 || local_->data[li] < 253;
  }

  // Polygon samples already include the body size. Only physical/lethal cells
  // reject them; applying the inscribed band again would count the body twice.
  bool footprint_point_free(const Eigen::Vector3d &world) const {
    if(!ready_||!world.allFinite())return false;
    const auto gi=global_geometry_.index(world);if(gi<0)return false;
    const int g=global_->data[gi];if(g<0||g>=100)return false;
    const auto li=local_geometry_.index(world);
    return li<0||local_->data[li]<254;
  }

  bool segment_traversable(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const
  {
    if (!ready_ || !a.allFinite() || !b.allFinite()) return false;
    const double distance = (b - a).norm();
    if (!std::isfinite(distance) || distance / step_ > 100000) return false;
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / step_)));
    for (int i = 0; i <= steps; ++i)
      if (!traversable(a + (b-a) * (static_cast<double>(i) / steps))) return false;
    return true;
  }

private:
  struct Geometry
  {
    double resolution = 0;
    uint32_t width = 0, height = 0;
    Eigen::Isometry3d world_to_grid = Eigen::Isometry3d::Identity();
    bool prepare(double r, uint32_t w, uint32_t h, const geometry_msgs::msg::Pose &origin,
                 const Eigen::Isometry3d &world_to_frame, size_t count)
    {
      if (!std::isfinite(r) || r <= 0 || !w || !h || uint64_t(w)*h != count ||
          !world_to_frame.matrix().allFinite()) return false;
      const auto &q = origin.orientation; const auto &p = origin.position;
      Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
      if (!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1) > 1e-3 ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
      Eigen::Isometry3d grid_to_frame = Eigen::Isometry3d::Identity();
      grid_to_frame.linear() = rotation.normalized().toRotationMatrix();
      grid_to_frame.translation() = Eigen::Vector3d(p.x,p.y,p.z);
      world_to_grid = grid_to_frame.inverse() * world_to_frame;
      resolution=r; width=w; height=h;
      return true;
    }
    int64_t index(const Eigen::Vector3d &world) const
    {
      const auto p = world_to_grid * world;
      const double x = std::floor(p.x()/resolution), y = std::floor(p.y()/resolution);
      if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 || x >= width || y >= height)
        return -1;
      return static_cast<int64_t>(y)*width + static_cast<int64_t>(x);
    }
  };
  Geometry global_geometry_, local_geometry_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr global_;
  nav2_msgs::msg::Costmap::ConstSharedPtr local_;
  bool ready_ = false;
  double step_ = .01;
};
}  // namespace rover_explore
