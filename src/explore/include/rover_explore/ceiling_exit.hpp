// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>
namespace rover_explore {
// Input is raw-lidar returns transformed with the synchronized pose into the
// gravity-aligned odometry frame. Height is relative to the lidar origin,
// never raw tilted-sensor Z and never estimated ground height.
class CeilingExit {
public:
  using Point=Eigen::Vector3d;
  using GroundHeight=std::function<std::optional<double>(const Point&)>;
  struct Parameters {
    double radius=2.,forward_min=-.5,forward_max=2.,half_width=1.,cell_size=.25;
    double minimum_relative_height=.3,height_threshold=2.,max_height=12.,minimum_elevation_degrees=30.;
    double minimum_high_fraction=.55;
    double confirm_time=.8,maximum_gap=.5;
    int minimum_cloud_points=100,minimum_upper_points=20,minimum_upper_cells=3,minimum_cell_points=2;
    void validate()const {
      for(double v:{radius,half_width,cell_size,minimum_relative_height,height_threshold,max_height,confirm_time,maximum_gap})
        if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid upper-return exit parameter");
      if(!std::isfinite(minimum_high_fraction)||minimum_high_fraction<=0||minimum_high_fraction>1)
        throw std::invalid_argument("Invalid high-return fraction");
      if(!std::isfinite(forward_min)||!std::isfinite(forward_max)||forward_min>=forward_max||
         minimum_relative_height>=height_threshold||height_threshold>=max_height||
         !std::isfinite(minimum_elevation_degrees)||minimum_elevation_degrees<=0||minimum_elevation_degrees>=90||
         minimum_cloud_points<1||minimum_upper_points<1||minimum_upper_cells<1||minimum_cell_points<1)
        throw std::invalid_argument("Inconsistent upper-return exit parameters");
    }
  }parameters;
  struct Detection {Point position;double yaw;};
  struct Statistics {
    int total=0,upper_points=0,upper_cells=0,low_points=0,high_points=0;
    double minimum_height=0,maximum_height=0,confirm_seconds=0,high_fraction=0;
    bool enough_returns=false,candidate=false,detected=false;
  }statistics;
  void reset(){last_.reset();high_since_.reset();detection_.reset();statistics={};}
  void invalidate(){high_since_.reset();statistics={};}
  const std::optional<Detection>& detection()const{return detection_;}
  void update(const std::vector<Point>& world,const Point& position,double yaw,double time,
              const GroundHeight& /*unused*/,std::optional<Point> sensor_origin=std::nullopt) {
    const Point origin=sensor_origin.value_or(position);
    if(!position.allFinite()||!origin.allFinite()||!std::isfinite(yaw)||!std::isfinite(time)){reset();return;}
    if(detection_)return;
    if(last_&&time<=*last_)reset();
    if(last_&&time-*last_>parameters.maximum_gap)invalidate();
    last_=time;statistics={};
    std::map<std::pair<int,int>,int> cells;
    const double c=std::cos(yaw),s=std::sin(yaw);
    const double slope=std::tan(parameters.minimum_elevation_degrees*std::acos(-1.)/180.);
    for(const auto& p:world) {
      if(!p.allFinite())continue;
      ++statistics.total;
      const auto d=(p-origin).eval();
      const double x=c*d.x()+s*d.y(),y=-s*d.x()+c*d.y(),height=d.z(),range=std::hypot(x,y);
      if(x<parameters.forward_min||x>parameters.forward_max||std::abs(y)>parameters.half_width||range>parameters.radius||
         height<parameters.minimum_relative_height||height>parameters.max_height||height<range*slope)continue;
      if(statistics.upper_points==0)statistics.minimum_height=statistics.maximum_height=height;
      statistics.minimum_height=std::min(statistics.minimum_height,height);
      statistics.maximum_height=std::max(statistics.maximum_height,height);
      ++statistics.upper_points;
      ++cells[{int(std::floor(x/parameters.cell_size)),int(std::floor(y/parameters.cell_size))}];
      // Exactly 2m remains low; accept a configurable majority above 2m.
      if(height<=parameters.height_threshold)++statistics.low_points;else ++statistics.high_points;
    }
    for(const auto& cell:cells)if(cell.second>=parameters.minimum_cell_points)++statistics.upper_cells;
    statistics.enough_returns=statistics.total>=parameters.minimum_cloud_points&&
      statistics.upper_points>=parameters.minimum_upper_points&&statistics.upper_cells>=parameters.minimum_upper_cells;
    statistics.high_fraction=statistics.upper_points?double(statistics.high_points)/statistics.upper_points:0.;
    statistics.candidate=statistics.enough_returns&&statistics.high_fraction>=parameters.minimum_high_fraction;
    if(!statistics.candidate){high_since_.reset();return;}
    if(!high_since_)high_since_=time;
    statistics.confirm_seconds=time-*high_since_;
    if(statistics.confirm_seconds>=parameters.confirm_time){detection_=Detection{position,yaw};statistics.detected=true;}
  }
private:
  std::optional<double> last_,high_since_;
  std::optional<Detection>detection_;
};
}
