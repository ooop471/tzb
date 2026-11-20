// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include "rover_explore/corridor_stability.hpp"
#include "rover_explore/corridor_dead_end.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace rover_explore
{
struct CorridorParameters
{
  bool enabled=false;
  double nominal_width=1.1, minimum_width=.70, maximum_width=1.55;
  double transition_width=2.4,wall_reference_height=.45,sensor_height=.20;
  double section_step=.20, section_half_length=.22, horizon=3.0;
  double entry_lookahead=.8, lookahead=1.2, center_band=.14;
  double target_min_distance=.2, target_max_distance=2.0;
  double body_half_width=.32, side_margin=.03;
  double handoff_distance=.35, hold_time=1.5, progress_timeout=8;
  int confirmation_cycles=2;
  double center_smoothing_time=.8,center_max_update=.04,center_jump_threshold=.12,max_heading_degrees=10.;
  int center_confirmation_cycles=3;
  bool exit_enabled=true;
  bool dead_end_enabled=true;
  double dead_end_distance=2.0;
  double body_front=.22, body_rear=.60, exit_margin=.05;
  double exit_extension=3.0, exit_lookahead=.8, exit_forward_distance=2.0;
  void validate() const
  {
    for (double x : {nominal_width,minimum_width,maximum_width,transition_width,wall_reference_height,sensor_height,section_step,section_half_length,
         horizon,entry_lookahead,lookahead,center_band,body_half_width,side_margin,
         center_smoothing_time,center_max_update,center_jump_threshold,max_heading_degrees,
         target_min_distance,target_max_distance,handoff_distance,hold_time,progress_timeout,body_front,body_rear,exit_margin,
         exit_extension,exit_lookahead,exit_forward_distance,dead_end_distance})
      if (!std::isfinite(x) || x<=0) throw std::invalid_argument("corridor parameters must be finite and positive");
    if (target_min_distance>=target_max_distance || target_max_distance>horizon ||
        minimum_width>=nominal_width || nominal_width>=maximum_width || maximum_width>=transition_width ||
        minimum_width+1e-6<2*(body_half_width+side_margin) || entry_lookahead<=handoff_distance ||
        lookahead<entry_lookahead || horizon<=lookahead || confirmation_cycles<1 || center_confirmation_cycles<1 || max_heading_degrees>45 ||
        exit_extension<=exit_forward_distance+handoff_distance || exit_lookahead<=handoff_distance ||
        dead_end_distance<.5 || dead_end_distance>horizon)
      throw std::invalid_argument("corridor widths, lookahead or confirmation cycles are inconsistent");
  }
};

// Uses already filtered obstacle points. This policy never changes a costmap,
// sensor filter or velocity; it supplies one observed, connected navigation goal.
class CorridorCenterline
{
public:
  using Point=Eigen::Vector3d;
  using Validator=std::function<bool(const Point&)>;
  using GroundHeight=std::function<std::optional<double>(const Point&)>;
  struct Section {Point center; double width;};
  CorridorParameters parameters;
  void reset()
  {
    axis_.reset(); last_time_.reset(); last_fit_.reset(); active_.reset();
    last_robot_.reset();stability_.reset();
    sections_.clear(); eligible_.clear(); confirmations_=0; entered_=false; inconsistent_=0;
    exit_.reset();
    exited_=false; last_selected_.reset();fit_source_="WAITING_FOR_INNER_WALLS";
    end_observation_={};
  }
  const std::vector<Section>& sections() const { return sections_; }
  const std::vector<Point>& eligible_candidates() const { return eligible_; }
  const std::string& fit_source() const { return fit_source_; }
  void release_target(){active_.reset();}
  const CorridorEndObservation& end_observation() const {return end_observation_;}
  // Explicitly requested only after a stopped, completed mission turnaround.
  // Ordinary grasp pauses retain the established direction as before.
  void reorient(const Point &robot,double yaw) {
    const bool was_entered=entered_;
    reset();axis_=yaw;entry_axis_=yaw;entry_origin_=robot;entered_=was_entered;
  }
  bool entered() const { return entered_; }
  bool exiting() const { return exit_.has_value() && !exited_; }
  bool exited() const { return exited_; }
  // Raw-ceiling detection fixes the origin and forward axis of the post-exit leg.
  void begin_cloud_exit(const Point &position,double yaw) {
    if(!parameters.exit_enabled||exit_||exited_)return;
    const Eigen::Vector2d forward(std::cos(yaw),std::sin(yaw));
    exit_=Exit{position,forward,parameters.nominal_width};active_.reset();
    fit_source_="EXIT_EXTENSION";
    Section a{position,parameters.nominal_width},b=a;
    a.center.head<2>()-=2*forward;
    b.center.head<2>()+=parameters.exit_extension*forward;
    sections_={a,b};
  }

  bool contains(const Point &p) const
  {
    const auto projection=project(p);
    return projection && projection->distance<=band(projection->width);
  }
  std::optional<Point> update(const Point &robot,double yaw,double time,
      const std::vector<Point> &obstacles,const std::vector<Point> &observed_candidates,
      const Validator &navigation_valid,const Validator &support={},const GroundHeight &ground_height={},
      const CorridorDeadEnd::Reader &physical_cell={})
  {
    eligible_.clear();
    end_observation_={};
    const Validator valid=[&](const Point&p){return navigation_valid(p)&&(!support||support(p));};
    const Validator route_valid=[&](const Point&p){
      if(!valid(p))return false;
      if(support) {
        const double distance=(p-robot).head<2>().norm();
        // Inside 0.6 m the existing self-filter hides the floor. Beyond that
        // blind region, never connect a target across a gap in floor evidence.
        for(double s=.6;s<distance;s+=.10)if(!support(robot+(p-robot)*(s/distance)))return false;
      }
      return true;
    };
    if (!robot.allFinite() || !std::isfinite(yaw) || !std::isfinite(time)) {reset();return {};}
    if(last_time_ && time<*last_time_)reset();
    if(last_time_ && time-*last_time_>3) {
      if(last_robot_ && (robot-*last_robot_).head<2>().norm()<.5) {
        // Grasp pause: retain the observed world-frame corridor and cave phase.
        if(last_fit_)last_fit_=time;
      } else reset();
    }
    last_time_=time;last_robot_=robot;
    if (exited_) return {};
    if (exit_) return exit_goal(robot,yaw,observed_candidates,route_valid);
    if (!axis_) {axis_=yaw;entry_origin_=robot;entry_axis_=yaw;}
    entered_ |= (robot-entry_origin_).head<2>().dot(Eigen::Vector2d(std::cos(entry_axis_),std::sin(entry_axis_)))>.6;
    std::vector<Point> low_walls;
    if(support) {
      for(const auto&p:obstacles) {
        // Production supplies the measured floor. The nominal height remains
        // only for standalone geometry fixtures without a ground estimator.
        const auto floor=ground_height?ground_height(p):std::optional<double>(robot.z()-parameters.sensor_height);
        if(floor&&p.z()-*floor>.05&&p.z()-*floor<parameters.wall_reference_height)low_walls.push_back(p);
      }
    }
    const auto &walls=support?low_walls:obstacles;
    // Visible floor can continue through the entrance into the room. It checks
    // target support only; it must not define the cave's lateral boundaries.
    auto fit=estimate(robot,*axis_,walls);
    const bool measured_pair=!fit.empty();
    fit_source_=fit.empty()?"WAITING_FOR_INNER_WALLS":"INNER_WALL_PAIR";
    if(fit.empty() && !sections_.empty() && last_fit_ && time-*last_fit_<=parameters.hold_time) {
      fit=estimate_one_wall(robot,*axis_,walls);
      fit_source_=fit.empty()?"HELD_INNER_WALLS":"ONE_WALL_KNOWN_WIDTH";
    }
    if (!fit.empty())
    {
      if(!sections_.empty() && last_fit_ && time-*last_fit_<=parameters.hold_time) {
        const double dt=std::clamp(time-*last_fit_,.02,1.);
        fit=stability_.update(std::move(fit),sections_,*axis_,dt,
          parameters.center_smoothing_time,parameters.center_max_update,
          parameters.center_jump_threshold,parameters.center_confirmation_cycles);
      }
      // Smooth cross-track estimates over ~0.8 m before deriving tangents.
      // A single missing floor strip must not create a 45..90 degree kink.
      if(fit.size()>=3) {
        const Eigen::Vector2d forward(std::cos(*axis_),std::sin(*axis_));
        const Eigen::Vector2d left(-forward.y(),forward.x());
        std::vector<double> along,side;
        for(const auto&section:fit){const auto delta=(section.center-robot).head<2>();along.push_back(delta.dot(forward));side.push_back(delta.dot(left));}
        auto filtered=side;
        for(size_t i=0;i<side.size();++i) {
          double sum=0,weight=0;
          for(size_t j=0;j<side.size();++j) {
            const double distance=std::abs(along[j]-along[i]);
            if(distance>.45)continue;
            const double w=1-distance/.5;sum+=w*side[j];weight+=w;
          }
          filtered[i]=sum/weight;
        }
        const double slope=std::tan(parameters.max_heading_degrees*3.141592653589793/180.);
        for(size_t i=1;i<filtered.size();++i) {
          const double limit=slope*std::max(.01,along[i]-along[i-1]);
          filtered[i]=std::clamp(filtered[i],filtered[i-1]-limit,filtered[i-1]+limit);
        }
        for(size_t i=filtered.size()-1;i>0;--i) {
          const double limit=slope*std::max(.01,along[i]-along[i-1]);
          filtered[i-1]=std::clamp(filtered[i-1],filtered[i]-limit,filtered[i]+limit);
        }
        for(size_t i=0;i<fit.size();++i) {
          const double shift=filtered[i]-side[i];fit[i].center.head<2>()+=shift*left;
          fit[i].width=std::max(.10,fit[i].width-2*std::abs(shift));
        }
      }
      sections_=fit;last_fit_=time;++confirmations_;
      // Follow the local tangent in the world frame, rather than the current
      // body yaw. A noisy body rotation cannot turn a side opening into forward.
      const auto d=sections_[std::min<size_t>(3,sections_.size()-1)].center-sections_.front().center;
      if (d.head<2>().norm()>.2)
        *axis_+=std::clamp(.25*wrap(std::atan2(d.y(),d.x())-*axis_),-.035,.035);
    }
    if (fit.empty() && (!last_fit_ || time-*last_fit_>parameters.hold_time))
    {
      sections_.clear();confirmations_=0;active_.reset();inconsistent_=0;return {};
    }
    const auto origin=project(robot);
    if(parameters.dead_end_enabled && measured_pair && origin && physical_cell && support) {
      // The local observed wall pair supplies a frame/width only. Neither the
      // last fitted section nor its extrapolated endpoint supplies cap evidence.
      end_observation_=CorridorDeadEnd::detect(origin->foot,*axis_,origin->width,
          parameters.dead_end_distance,2*(parameters.body_half_width+parameters.side_margin),
          walls,physical_cell,support);
    }
    if (origin && confirmations_>=parameters.confirmation_cycles && sections_.size()>=3) {
      // Sample the measured center itself, independent of the rolling viewpoint
      // lattice. These are path endpoints, not old TARE gain/viewpoint targets.
      double arc=0;
      for(size_t i=1;i<sections_.size();++i) {
        const auto&a=sections_[i-1];const auto&b=sections_[i];
        const auto delta=b.center-a.center;const double length=delta.head<2>().norm();
        if(length<1e-6)continue;
        for(double u=0;u<length+1e-6;u+=.05) {
          Point p=a.center+std::min(u/length,1.)*delta;p.z()=robot.z();
          const double advance=arc+u-origin->arc;
          if(advance>=parameters.target_min_distance && advance<=parameters.target_max_distance &&
             (p-robot).head<2>().norm()>=parameters.target_min_distance && valid(p))eligible_.push_back(p);
        }
        arc+=length;
      }
    }
    if (active_)
    {
      const auto reference=project(*active_);
      if(reference){*active_=reference->foot;active_->z()=robot.z();}
      const double distance=(*active_-robot).head<2>().norm();
      if (distance<=parameters.handoff_distance || !valid(*active_)) active_.reset();
      else
      {
        if (best_distance_-distance>=.1) {best_distance_=distance;progress_time_=time;}
        const auto p=project(*active_);
        if (!p || p->distance>band(p->width)+.08) ++inconsistent_; else inconsistent_=0;
        if (inconsistent_<parameters.confirmation_cycles && time-progress_time_<parameters.progress_timeout)
          return active_;
        active_.reset();
      }
    }
    if (confirmations_<parameters.confirmation_cycles || sections_.size()<3) return {};
    if (!origin) return {};
    const double desired_join=std::min(parameters.target_max_distance,
      .8+5*origin->distance+1.2*std::abs(wrap(yaw-*axis_)));
    const double lookahead=std::max(entered_ ? parameters.lookahead : parameters.entry_lookahead,desired_join);
    double best=1e9;std::optional<Point> chosen;
    for (const auto &p:eligible_)
    {
      const auto v=project(p);
      if (!v || v->distance>band(v->width)) continue;
      const double advance=v->arc-origin->arc;
      double score=1.5*std::abs(advance-lookahead)+8*v->distance;
      // Prefer the same side of equally near grid samples across handoffs.
      if (last_selected_)
      {
        const auto prior=project(*last_selected_);
        if (prior && prior->distance<.25)
        {
          const Eigen::Vector2d left(-std::sin(*axis_),std::cos(*axis_));
          score+=2*std::abs(((p-v->foot)-(*last_selected_-prior->foot)).head<2>().dot(left));
        }
      }
      if (score<best) {best=score;chosen=p;}
    }
    if (chosen)
    {
      active_=chosen;best_distance_=(*chosen-robot).head<2>().norm();progress_time_=time;inconsistent_=0;
      last_selected_=chosen;
    }
    return chosen;
  }
private:
  struct Exit {Point mouth;Eigen::Vector2d forward;double width;};
  std::optional<Point> exit_goal(const Point &robot,double yaw,
      const std::vector<Point>& observed,const Validator& valid)
  {
    (void)yaw;
    const auto &exit=*exit_;const Eigen::Vector2d left(-exit.forward.y(),exit.forward.x());
    const double along=(robot-exit.mouth).head<2>().dot(exit.forward);
    // Actual forward displacement since cloud-drop confirmation, independent of speed/time.
    if(along>=parameters.exit_forward_distance)
    {exited_=true;active_.reset();return {};}
    if(active_ && valid(*active_) && (*active_-robot).head<2>().norm()>parameters.handoff_distance)
      return active_;
    active_.reset();double best=1e9;
    const double desired=std::min(along+parameters.exit_lookahead,parameters.exit_extension);
    for(const auto &p:observed)
    {
      if(!p.allFinite() || !valid(p)) continue;
      const Eigen::Vector2d d=(p-exit.mouth).head<2>();
      const double x=d.dot(exit.forward),y=d.dot(left);
      if(x-along<.4 || x>parameters.exit_extension || std::abs(y)>parameters.center_band) continue;
      const double score=1.5*std::abs(x-desired)+8*std::abs(y);
      if(score<best) {best=score;active_=p;}
    }
    return active_;
  }
  std::vector<Point> eligible_;
  struct Projection {double distance,arc,width;Point foot;};
  static double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
  double band(double width) const
  {return std::max(0.0,std::min(parameters.center_band,width*.5-parameters.body_half_width-parameters.side_margin));}
  std::optional<Projection> project(const Point &p) const
  {
    if (!p.allFinite() || sections_.size()<2) return {};
    Projection best{1e9,0,0,Point::Zero()};double arc=0;
    for(size_t i=1;i<sections_.size();++i)
    {
      const auto &a=sections_[i-1];const auto &b=sections_[i];
      const Eigen::Vector2d d=(b.center-a.center).head<2>();const double length=d.norm();
      if (length<1e-6) continue;
      const double t=std::clamp((p-a.center).head<2>().dot(d)/(length*length),i==1?-.95/length:0.0,1.0);
      const double distance=((p-a.center).head<2>()-t*d).norm();
      if (distance<best.distance)
        best={distance,arc+t*length,a.width+t*(b.width-a.width),a.center+t*(b.center-a.center)};
      arc+=length;
    }
    return best.distance<1e8 ? std::optional<Projection>(best) : std::nullopt;
  }
  std::vector<Section> estimate(const Point &robot,double axis,const std::vector<Point> &cloud) const
  {
    std::vector<Section> best;double best_score=-1e9;
    for (int angle=-25;angle<=25;angle+=5)
    {
      const double heading=axis+angle*3.141592653589793/180;
      const Eigen::Vector2d forward(std::cos(heading),std::sin(heading));
      const Eigen::Vector2d left(-forward.y(),forward.x());
      std::vector<Section> line;double previous=0,quality=0;int missing=0;
      for(double s=.3;s<=parameters.horizon+1e-6;s+=parameters.section_step)
      {
        std::vector<double> lateral;
        for(const auto &p:cloud)
        {
          if (!p.allFinite()) continue;
          const Eigen::Vector2d delta=(p-robot).head<2>();
          const double side=delta.dot(left);
          if (std::abs(delta.dot(forward)-s)<=parameters.section_half_length && std::abs(side)<1.3)
            lateral.push_back(side);
        }
        struct Peak {double position;int support;};std::vector<Peak> peaks;
        for(double b=-1.24;b<=1.24;b+=.08)
        {
          std::vector<double> members;
          for(double v:lateral) if(std::abs(v-b)<.08) members.push_back(v);
          if (members.size()<2) continue;
          std::sort(members.begin(),members.end());
          const double mid=members[members.size()/2];
          if (peaks.empty() || std::abs(peaks.back().position-mid)>.06)
            peaks.push_back({mid,static_cast<int>(members.size())});
          else if (static_cast<int>(members.size())>peaks.back().support)
            peaks.back()={mid,static_cast<int>(members.size())};
        }
        double pair_score=-1e9,mid=0,width=0;
        for(const auto &r:peaks) for(const auto &l:peaks)
        {
          const double w=l.position-r.position,c=(l.position+r.position)*.5;
          if (w<parameters.minimum_width || w>parameters.maximum_width || std::abs(c)>.6 ||
              (!line.empty() && std::abs(c-previous)>.22)) continue;
          const double score=.2*std::log1p(std::min(r.support,l.support))-
              4*std::abs(w-parameters.nominal_width)-2*std::abs(c-previous);
          if(score>pair_score) {pair_score=score;mid=c;width=w;}
        }
        if(width==0)
        {
          if(++missing>2 && !line.empty()) break;
          continue;
        }
        if(line.empty() && s>1.4) break;
        missing=0;previous=mid;quality+=pair_score;
        Point center=robot;center.head<2>()+=s*forward+mid*left;
        line.push_back({center,width});
      }
      // A narrow inner-wall run is the reference even when the floor outside
      // the entrance is wider. Do not match two isolated posts as a tunnel.
      if(line.size()<2) continue;
      if(!has_wall_extent(line,cloud,forward,left,true))continue;
      const double score=2*line.size()+quality/line.size()-.025*std::abs(angle);
      if(score>best_score) {best_score=score;best=std::move(line);}
    }
    if(best.size()>=2)
    {
      auto original=best;
      for(size_t i=1;i+1<best.size();++i)
        best[i].center=.25*original[i-1].center+.5*original[i].center+.25*original[i+1].center;
      if(best.size()==2)
      {
        // Extend a short entrance hypothesis by at most 0.6 m. This does not
        // create free space: every selected target still requires sensor and
        // navigation evidence supplied by the caller.
        const auto delta=best.back().center-best.front().center;
        const double direction=std::atan2(delta.y(),delta.x());
        const double heading=axis+std::clamp(wrap(direction-axis),-.35,.35);
        Section end=best.back();
        end.center+=Point(.6*std::cos(heading),.6*std::sin(heading),0);
        best.push_back(end);
      }
      // Extrapolate the first measured center back to the robot's longitudinal
      // station, so a short entrance preview is measured along the whole route.
      const auto d=best[1].center-best[0].center;
      if(d.head<2>().norm()>.05)
      {
        const double along=(best[0].center-robot).head<2>().dot(d.head<2>().normalized());
        Section start=best[0];start.center-=std::max(0.0,along)*d/d.head<2>().norm();
        best.insert(best.begin(),start);
      }
    }
    return best;
  }
  // Require longitudinal wall evidence, not repeated views of one post in
  // overlapping section windows. The width reference comes from paired walls.
  bool has_wall_extent(const std::vector<Section>& line,const std::vector<Point>& cloud,
      const Eigen::Vector2d& forward,const Eigen::Vector2d& left,bool both) const {
    if(line.size()<2)return false;
    double lo[2]={1e9,1e9},hi[2]={-1e9,-1e9};
    for(const auto&p:cloud) {
      if(!p.allFinite())continue;
      const Section* nearest=nullptr;double distance=parameters.section_half_length;
      for(const auto&s:line) {
        const double d=std::abs((p-s.center).head<2>().dot(forward));
        if(d<=distance){distance=d;nearest=&s;}
      }
      if(!nearest)continue;
      const double side=(p-nearest->center).head<2>().dot(left);
      for(int k=0;k<2;++k)if(std::abs(side-(k?1:-1)*nearest->width*.5)<.12) {
        const double along=p.head<2>().dot(forward);lo[k]=std::min(lo[k],along);hi[k]=std::max(hi[k],along);
      }
    }
    const bool right=hi[0]-lo[0]>=.4,left_wall=hi[1]-lo[1]>=.4;
    return both?(right&&left_wall):(right||left_wall);
  }
  std::vector<Section> estimate_one_wall(const Point&robot,double axis,const std::vector<Point>&cloud) const {
    const Eigen::Vector2d forward(std::cos(axis),std::sin(axis)),left(-forward.y(),forward.x());
    std::vector<Section> fit;int missing=0;
    for(double along=.3;along<=parameters.horizon+1e-6;along+=parameters.section_step) {
      Point base=robot;base.head<2>()+=along*forward;
      const auto prior=project(base);
      if(!prior||prior->distance>.8||prior->width>parameters.maximum_width)continue;
      const double center=(prior->foot-base).head<2>().dot(left);
      std::vector<double> side_points[2];
      for(const auto&p:cloud) {
        if(!p.allFinite()||std::abs((p-base).head<2>().dot(forward))>parameters.section_half_length)continue;
        const double side=(p-base).head<2>().dot(left);
        for(int k=0;k<2;++k)if(std::abs(side-center-(k?1:-1)*prior->width*.5)<.16)side_points[k].push_back(side);
      }
      double inferred=0;int count=0;
      for(int k=0;k<2;++k)if(side_points[k].size()>=2) {
        auto &points=side_points[k];std::sort(points.begin(),points.end());
        inferred+=points[points.size()/2]-(k?1:-1)*prior->width*.5;++count;
      }
      if(count==0){if(++missing>2&&!fit.empty())break;continue;}
      missing=0;base.head<2>()+=(inferred/count)*left;fit.push_back({base,prior->width});
    }
    return has_wall_extent(fit,cloud,forward,left,false)?fit:std::vector<Section>{};
  }
  CorridorFitStability stability_;
  CorridorEndObservation end_observation_;
  std::optional<Point> last_robot_;
  std::string fit_source_="WAITING_FOR_INNER_WALLS";
  std::vector<Section> sections_;
  std::optional<double> axis_,last_time_,last_fit_;
  std::optional<Point> active_;
  Point entry_origin_=Point::Zero();double entry_axis_=0;
  double best_distance_=0,progress_time_=0;
  int confirmations_=0,inconsistent_=0;
  bool entered_=false;
  std::optional<Exit> exit_;
  std::optional<Point> last_selected_;
  bool exited_=false;
};
} // namespace rover_explore
