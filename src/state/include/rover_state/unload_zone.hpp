#pragma once
#include "rover_state/geometry.hpp"
#include <algorithm>

namespace rover_state {
// A point on the chassis, not the chassis origin or its entire rear footprint.
struct UnloadZone {
  Point reference_offset{-.40,0.,0.};
  double width{.5},height{.5},margin{.05};
  void validate() const {
    if(!finite(reference_offset)||!std::isfinite(width)||!std::isfinite(height)||
       !std::isfinite(margin)||width<=0||height<=0||margin<0||2*margin>=std::min(width,height))
      throw std::invalid_argument("Invalid unload point/zone geometry");
  }
  Point reference(Pose base) const {
    return {base.p.x+std::cos(base.yaw)*reference_offset.x-std::sin(base.yaw)*reference_offset.y,
            base.p.y+std::sin(base.yaw)*reference_offset.x+std::cos(base.yaw)*reference_offset.y,base.p.z};
  }
  Point point_error(Point point,Pose zone) const {
    const double dx=point.x-zone.p.x,dy=point.y-zone.p.y;
    return {std::cos(zone.yaw)*dx+std::sin(zone.yaw)*dy,
            -std::sin(zone.yaw)*dx+std::cos(zone.yaw)*dy,0.};
  }
  Point error(Pose base,Pose zone) const {return point_error(reference(base),zone);}
  bool contains_point(Point point,Pose zone) const {
    if(!finite(point)||!finite(zone.p)||!std::isfinite(zone.yaw))return false;
    const auto e=point_error(point,zone);
    return std::abs(e.x)<=width/2-margin && std::abs(e.y)<=height/2-margin;
  }
  bool contains(Pose base,Pose zone) const {
    return std::isfinite(base.yaw)&&contains_point(reference(base),zone);
  }
  // Heading is chosen for navigation, never an unloading acceptance condition.
  Pose base_goal(Pose zone,double heading) const {
    Pose goal{zone.p,heading};const auto offset=reference({{0.,0.,0.},heading});
    goal.p.x-=offset.x;goal.p.y-=offset.y;return goal;
  }
};
}
