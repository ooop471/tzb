// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <vector>

namespace rover_explore
{
struct GoalSelectionParameters
{
  bool enabled = true;
  double main_cone_degrees = 35.0;
  double max_turn_degrees = 85.0;
  double min_distance = 0.4;
  double max_distance = 4.0;
  double target_distance = 2.0;
  double handoff_distance = 0.6;
  int empty_cycles = 3;
  double progress_distance = 0.15;
  double progress_timeout = 8.0;
  double retry_cooldown = 12.0;
  double retry_radius = 0.6;
  void validate() const
  {
    for (double v : {main_cone_degrees,max_turn_degrees,min_distance,max_distance,
                     target_distance,handoff_distance,progress_distance,progress_timeout,
                     retry_cooldown,retry_radius})
      if (!std::isfinite(v) || v<=0) throw std::invalid_argument("goal_selection values must be finite and positive");
    if (main_cone_degrees>max_turn_degrees || max_turn_degrees>=90 ||
        handoff_distance>=target_distance || min_distance>=max_distance ||
        target_distance<min_distance || target_distance>max_distance || empty_cycles<1)
      throw std::invalid_argument("goal_selection angles, distances or empty_cycles are inconsistent");
  }
};
struct GoalOption
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double gain = 0; // max(surface/min_surface, frontier/min_frontier)
  double path_cost = 0;
};
enum class GoalMode { FORWARD, BRANCH, RELOCATE };
struct SelectedGoal
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  GoalMode mode = GoalMode::FORWARD;
};
inline const char *goal_mode_name(GoalMode mode)
{
  return mode==GoalMode::FORWARD ? "FORWARD" : mode==GoalMode::BRANCH ? "BRANCH" : "RELOCATE";
}
// Only chooses goals. This class never publishes velocities or cancels a controller.
class GoalSelection
{
public:
  GoalSelectionParameters parameters;
  using Validator = std::function<bool(const Eigen::Vector3d&)>;
  void reset()
  {
    active_.reset();blocked_.clear();last_time_.reset();heading_.reset();empty_count_=0;
  }
  const std::optional<SelectedGoal>& active() const { return active_; }
  std::optional<SelectedGoal> update(const Eigen::Vector3d &robot, double yaw, double time,
      const std::vector<GoalOption> &local, const std::vector<GoalOption> &global,
      const Validator &valid)
  {
    if (!robot.allFinite() || !std::isfinite(yaw) || !std::isfinite(time)) {reset();return {};}
    if (last_time_ && (time<*last_time_ || time-*last_time_>3.0)) reset();
    last_time_=time;
    if (!heading_) heading_=yaw;
    blocked_.erase(std::remove_if(blocked_.begin(),blocked_.end(),
        [time](const Blocked &b){return time>=b.until;}),blocked_.end());
    const auto distance=[&](const Eigen::Vector3d &p){return (p-robot).head<2>().norm();};
    const auto available=[&](const GoalOption &o) {
      if (!o.position.allFinite() || !std::isfinite(o.gain) || !std::isfinite(o.path_cost)) return false;
      if (distance(o.position)<parameters.min_distance || distance(o.position)>parameters.max_distance) return false;
      return true;
    };
    const auto cooled=[&](const GoalOption &o) {
      for (const auto &b:blocked_)
        if ((b.position-o.position).head<2>().norm()<parameters.retry_radius) return true;
      return false;
    };
    std::vector<GoalOption> forward,branch;
    for (const auto &o:local)
    {
      if (!available(o) || o.gain<1 || !valid(o.position)) continue;
      const auto d=o.position-robot;
      const double angle=std::abs(wrap(std::atan2(d.y(),d.x())-yaw))*180/pi;
      if (angle>parameters.max_turn_degrees) continue;
      if (angle<=parameters.main_cone_degrees) forward.push_back(o);
      else branch.push_back(o);
    }
    const bool fresh_work=std::any_of(forward.begin(),forward.end(),[&](const auto &o){return !cooled(o);}) ||
        std::any_of(branch.begin(),branch.end(),[&](const auto &o){return !cooled(o);});
    empty_count_=fresh_work ? 0 : empty_count_+1;
    if (active_)
    {
      const double d=distance(active_->position);
      if (d<=handoff_distance_) active_.reset();
      else if (!valid(active_->position)) active_.reset();
      else
      {
        if (best_distance_-d>=parameters.progress_distance)
        {best_distance_=d;progress_time_=time;}
        if (time-progress_time_>=parameters.progress_timeout)
        {
          blocked_.push_back({active_->position,time+parameters.retry_cooldown});
          active_.reset();
        }
        else return active_; // Do not chase a rotating FOV or a slightly different gain.
      }
    }
    const auto select=[&](std::vector<GoalOption> options, GoalMode mode, bool retry=false)->std::optional<SelectedGoal> {
      const auto score=[&](const GoalOption &o) {
        const auto d=o.position-robot;const double bearing=std::atan2(d.y(),d.x());
        const double turn=std::abs(wrap(bearing-*heading_));
        if (mode==GoalMode::RELOCATE) return -1.5*turn-.3*o.path_cost;
        const double advance=std::cos(bearing-*heading_)*distance(o.position);
        return std::log1p(std::min(o.gain,10.0))+.4*advance-
               .6*std::abs(distance(o.position)-parameters.target_distance)-1.5*turn;
      };
      std::stable_sort(options.begin(),options.end(),[&](const auto &a,const auto &b){return score(a)>score(b);});
      for (const auto &o:options)
      {
        if (!available(o) || (!retry && cooled(o)) || !valid(o.position)) continue;
        active_=SelectedGoal{o.position,mode};
        const auto d=o.position-robot;heading_=std::atan2(d.y(),d.x());
        best_distance_=distance(o.position);progress_time_=time;
        handoff_distance_=std::min(parameters.handoff_distance,best_distance_*.5);
        return active_;
      }
      return {};
    };
    if (auto chosen=select(forward,GoalMode::FORWARD)) return chosen;
    if (auto chosen=select(branch,GoalMode::BRANCH)) return chosen;
    if (empty_count_>=parameters.empty_cycles)
      if (auto chosen=select(global,GoalMode::RELOCATE)) return chosen;
    // Cooldown is a preference, not a hard endpoint blacklist or a forced wait.
    if (auto chosen=select(forward,GoalMode::FORWARD,true)) return chosen;
    if (auto chosen=select(branch,GoalMode::BRANCH,true)) return chosen;
    if (empty_count_>=parameters.empty_cycles) return select(global,GoalMode::RELOCATE,true);
    return {};
  }
private:
  static constexpr double pi=3.141592653589793;
  static double wrap(double v){return std::atan2(std::sin(v),std::cos(v));}
  struct Blocked {Eigen::Vector3d position;double until;};
  std::optional<SelectedGoal> active_;
  std::vector<Blocked> blocked_;
  std::optional<double> heading_,last_time_;
  int empty_count_=0;
  double best_distance_=0,progress_time_=0,handoff_distance_=0;
};
} // namespace rover_explore
