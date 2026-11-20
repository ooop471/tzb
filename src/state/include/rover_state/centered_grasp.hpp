#pragma once
#include "rover_state/geometry.hpp"
#include <algorithm>
#include <functional>
#include <limits>
#include <optional>

namespace rover_state {
struct CorridorProjection {double arc{},lateral{},yaw{},distance{};Point foot;};
inline std::optional<CorridorProjection> corridor_projection(const std::vector<Pose>& line,Point p) {
  if(line.size()<2 || !finite(p)) return {};
  double arc=0;CorridorProjection best;best.distance=std::numeric_limits<double>::infinity();
  for(size_t i=1;i<line.size();++i) {
    const Point a=line[i-1].p,b=line[i].p;const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
    if(length<1e-5) continue;
    const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(length*length),0.,1.);
    const Point foot{a.x+t*dx,a.y+t*dy,a.z+t*(b.z-a.z)};const double d=distance(p,foot);
    if(d<best.distance) best={arc+t*length,(-(p.x-foot.x)*dy+(p.y-foot.y)*dx)/length,
                            std::atan2(dy,dx),d,foot};
    arc+=length;
  }
  return std::isfinite(best.distance) ? std::optional<CorridorProjection>(best) : std::nullopt;
}
inline Pose corridor_pose(const std::vector<Pose>& line,double station) {
  double arc=0;
  for(size_t i=1;i<line.size();++i) {
    const auto a=line[i-1].p,b=line[i].p;const double length=distance(a,b);
    if(length<1e-5) continue;
    if(arc+length>=station) {
      const double t=std::clamp((station-arc)/length,0.,1.);
      return {{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)},std::atan2(b.y-a.y,b.x-a.x)};
    }
    arc+=length;
  }
  return line.back();
}
struct CenteredGraspParameters {
  double front=.30,near_gap=.13,far_gap=.40,half_width=.14,margin=0.0,half_angle=35*M_PI/180.,preferred_radius=.30;
  double maximum_turn=.17453292519943295,maximum_offset=.14,step=.025;
  void validate() const {
    for(double v:{front,near_gap,far_gap,half_width,maximum_turn,maximum_offset,step,half_angle,preferred_radius})
      if(!std::isfinite(v) || v<=0) throw std::invalid_argument("Invalid centered grasp geometry");
    if(!std::isfinite(margin)||margin<0||near_gap+2*margin>=far_gap || preferred_radius<near_gap||preferred_radius>far_gap||half_angle>=M_PI/2 || maximum_turn>.5235987756)
      throw std::invalid_argument("Centered grasp margin or angular bound is inconsistent");
  }
  bool contains(Pose robot,Point target) const {
    return inside_grasp_sector(alignment(robot,target,front),near_gap+margin,far_gap-margin,half_angle);
  }
};
inline double grasp_approach_speed(double gap,double desired,double maximum) {
  // Keep the configured approach speed until the distance threshold is reached.
  return std::min(maximum,std::clamp(.5*std::max(0.,gap-desired),.10,.15));
}
using PoseValidator=std::function<bool(Pose)>;
// Straight correction to a comfortable longitudinal distance, respecting the
// measured annular sector. Lateral errors still use the bounded heading step.
inline std::optional<double> grasp_distance_correction(Pose robot,Point target,
    const CenteredGraspParameters& p,double tolerance) {
  const auto a=alignment(robot,target,p.front);
  const double lateral=std::abs(a.lateral),far=p.far_gap-p.margin,near=p.near_gap+p.margin;
  if(lateral>=far)return {};
  const double lo=std::max(lateral/std::tan(p.half_angle),
    std::sqrt(std::max(0.,near*near-lateral*lateral)));
  const double hi=std::sqrt(far*far-lateral*lateral);
  if(lo>hi)return {};
  const double want=std::clamp(p.preferred_radius,lo,hi);
  if(p.contains(robot,target)&&std::abs(a.gap-want)<=tolerance)return 0.;
  return a.gap-want; // positive forward; negative reverse
}
inline bool sweep_clear(Pose start,double yaw,const PoseValidator& valid) {
  const double turn=angle(yaw-start.yaw);const int n=std::max(1,static_cast<int>(std::ceil(std::abs(turn)/.02)));
  for(int i=0;i<=n;++i) {Pose p=start;p.yaw+=turn*i/n;if(!valid(p)) return false;}
  return true;
}
struct GraspAdjustment {double yaw{},translation{};};
inline std::optional<GraspAdjustment> joint_grasp_adjustment(Pose robot,double tangent,Point target,
    const CenteredGraspParameters& p,double budget,const PoseValidator& valid) {
  std::optional<GraspAdjustment> best;double best_cost=std::numeric_limits<double>::infinity();
  auto consider=[&](double yaw) {
    if(std::abs(angle(yaw-tangent))>p.maximum_turn+1e-8)return;
    Pose pose=robot;pose.yaw=yaw;
    auto correction=grasp_distance_correction(pose,target,p,0.);
    if(!correction || std::abs(*correction)>budget+1e-8 || !sweep_clear(robot,yaw,valid))return;
    const int steps=std::max(1,int(std::ceil(std::abs(*correction)/.02)));
    for(int i=0;i<=steps;++i) {
      Pose check=pose;const double d=*correction*i/steps;
      check.p.x+=d*std::cos(yaw);check.p.y+=d*std::sin(yaw);
      if(!valid(check))return;
    }
    // Prefer a central target, with a small penalty for motion; retain limits.
    const auto a=alignment(pose,target,p.front);
    const double cost=std::abs(a.lateral)+.2*std::abs(*correction)+.03*std::abs(angle(yaw-robot.yaw));
    if(cost<best_cost){best_cost=cost;best=GraspAdjustment{yaw,*correction};}
  };
  consider(robot.yaw);
  for(double turn=-p.maximum_turn;turn<=p.maximum_turn+1e-8;turn+=.008726646259971648)
    consider(angle(tangent+turn));
  consider(angle(tangent+p.maximum_turn));
  return best;
}
inline std::optional<double> minimum_grasp_yaw(Pose robot,double tangent,Point target,
    const CenteredGraspParameters& parameters,const PoseValidator& valid) {
  std::optional<double> chosen;double best=1e9;
  for(double turn=-parameters.maximum_turn;turn<=parameters.maximum_turn+1e-8;turn+=.008726646259971648) {
    Pose pose=robot;pose.yaw=angle(tangent+turn);
    if(!parameters.contains(pose,target)) continue;
    const double cost=std::abs(angle(pose.yaw-robot.yaw));
    if(cost<best && sweep_clear(robot,pose.yaw,valid)) {chosen=pose.yaw;best=cost;}
  }
  // Include the actual heading exactly, even when the parameter is not a multiple of the sample step.
  if(std::abs(angle(robot.yaw-tangent))<=parameters.maximum_turn && parameters.contains(robot,target) && valid(robot))
    return robot.yaw;
  return chosen;
}
// Predict a stopping station on the path ALREADY accepted by the exploration
// controller. This never builds/submits a replacement path or a joining segment.
struct RouteGraspStop {Pose travel;double grasp_yaw{},remaining{};};
inline std::optional<RouteGraspStop> route_grasp_stop(Pose robot,Point target,
    const std::vector<Pose>& route,const CenteredGraspParameters& parameters,const PoseValidator& valid) {
  auto origin=corridor_projection(route,robot.p);
  if(!origin || origin->distance>.30)return {};
  double length=0;for(size_t i=1;i<route.size();++i)length+=distance(route[i-1].p,route[i].p);
  std::optional<RouteGraspStop> result;double best=1e9;
  for(double arc=origin->arc;arc<=length+1e-8;arc+=parameters.step) {
    Pose pose=corridor_pose(route,arc);pose.p.z=robot.p.z;
    auto yaw=minimum_grasp_yaw(pose,pose.yaw,target,parameters,valid);
    if(!yaw)continue;
    Pose grasp=pose;grasp.yaw=*yaw;auto a=alignment(grasp,target,parameters.front);
    double cost=10*std::abs(angle(*yaw-pose.yaw))+
      std::abs(std::hypot(a.gap,a.lateral)-parameters.preferred_radius);
    if(cost<best){best=cost;result=RouteGraspStop{pose,*yaw,arc-origin->arc};}
  }
  return result;
}
} // namespace rover_state
