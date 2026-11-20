#pragma once
#include <cmath>
#include <stdexcept>
#include <vector>

namespace rover_state {
struct Point {
  double x{}, y{}, z{};
};
struct Pose {
  Point p;
  double yaw{};
};
inline double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }
inline double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline bool finite(Point p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
inline Pose stopping_pose(Pose robot, Point target, double front, double gap) {
  if (!finite(robot.p) || !finite(target) || !std::isfinite(front) || front < 0 ||
      !std::isfinite(gap) || gap <= 0 || distance(robot.p, target) < 1e-6)
    throw std::invalid_argument("Invalid stopping geometry");
  const double yaw = std::atan2(target.y - robot.p.y, target.x - robot.p.x);
  return {{target.x - (front + gap) * std::cos(yaw), target.y - (front + gap) * std::sin(yaw),
           robot.p.z},
          yaw};
}
struct Alignment {
  double gap, lateral, yaw;
};
inline Alignment alignment(Pose robot, Point target, double front) {
  const double dx = target.x - robot.p.x, dy = target.y - robot.p.y;
  return {dx * std::cos(robot.yaw) + dy * std::sin(robot.yaw) - front,
          -dx * std::sin(robot.yaw) + dy * std::cos(robot.yaw),
          angle(std::atan2(dy, dx) - robot.yaw)};
}
// Measured ground-plane grasp rectangle, referenced to the chassis front edge.
inline bool inside_grasp_region(const Alignment &target, double near_gap, double far_gap,
                                double half_width) {
  constexpr double epsilon = 1e-9;  // Floating-point boundary equality, not a geometric margin.
  return std::isfinite(target.gap) && std::isfinite(target.lateral) &&
         target.gap >= near_gap - epsilon && target.gap <= far_gap + epsilon &&
         std::abs(target.lateral) <= half_width + epsilon;
}
// Annular sector measured from the chassis front-edge centre, not base_frame.
inline bool inside_grasp_sector(const Alignment &target,double inner,double outer,double half_angle) {
  const double radius=std::hypot(target.gap,target.lateral);
  return std::isfinite(radius) && std::isfinite(half_angle) &&
    radius>=inner-1e-9 && radius<=outer+1e-9 &&
    std::abs(std::atan2(target.lateral,target.gap))<=half_angle+1e-9;
}
struct Sample {
  double stamp;
  Pose pose;
};
// Uses the complete window, never Odometry.twist (Point-LIO leaves it zero).
inline bool settled(const std::vector<Sample> &samples, double duration, double position,
                    double yaw) {
  if (samples.size() < 3 || samples.back().stamp - samples.front().stamp < duration) return false;
  const auto &first = samples.front();
  double previous = first.stamp;
  for (const auto &s : samples) {
    if (!finite(s.pose.p) || !std::isfinite(s.pose.yaw) || s.stamp < previous ||
        s.stamp - previous > 0.25 || distance(s.pose.p, first.pose.p) > position ||
        std::abs(angle(s.pose.yaw - first.pose.yaw)) > yaw)
      return false;
    previous = s.stamp;
  }
  return true;
}
}  // namespace rover_state
