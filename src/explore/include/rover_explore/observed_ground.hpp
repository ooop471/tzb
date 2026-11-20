// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <vector>
namespace rover_explore {
// Observation-only cache: does not alter registered scans, terrain or Nav2 maps.
// The odometry origin is not necessarily the physical lidar origin. Learn the
// floor in the registered cloud's world frame instead of subtracting lidar height.
class ObservedGround {
 public:
  // sensor_height is retained for configuration compatibility, not as floor Z.
  double sensor_height=.20,height_tolerance=.10,resolution=.10,radius=.14,max_age=4.0;
  double fit_radius=3.0,fit_tolerance=.035,max_slope_degrees=20.0;
  int minimum_inliers=12;
  void validate() const {
    for(double x:{sensor_height,height_tolerance,resolution,radius,max_age,fit_radius,fit_tolerance,max_slope_degrees})
      if(!std::isfinite(x)||x<=0)throw std::invalid_argument("Invalid ground observation parameters");
    if(height_tolerance>.15||radius>.25||fit_radius<1||fit_radius>5||fit_tolerance>height_tolerance||
       max_slope_degrees>30||minimum_inliers<6)
      throw std::invalid_argument("Ground fit tolerance, radius, slope or support count out of range");
  }
  void reset(){samples_.clear();cells_.clear();plane_.reset();last_time_=-1;}
  void update(const std::vector<Eigen::Vector3d>& points,const Eigen::Vector3d& robot,double time) {
    if(!robot.allFinite()||!std::isfinite(time)){reset();return;}
    if(last_time_>time)reset();last_time_=time;robot_=robot;
    for(auto it=samples_.begin();it!=samples_.end();) {
      if(time-it->second.time>max_age || (it->second.point-robot).head<2>().norm()>9.)it=samples_.erase(it);else ++it;
    }
    // One low representative per observed XY cell: walls cannot overwhelm the
    // fit merely by contributing many vertical returns at the same location.
    std::unordered_map<uint64_t,Cell> frame;
    for(const auto&p:points) {
      if(!p.allFinite()||(p-robot).head<2>().norm()>9.||std::abs(p.z()-robot.z())>1.5)continue;
      const auto k=key(int(std::floor(p.x()/resolution)),int(std::floor(p.y()/resolution)));
      auto it=frame.find(k);
      if(it==frame.end()||p.z()<it->second.point.z())frame[k]={p,time};
    }
    for(const auto&v:frame)samples_[v.first]=v.second;
    fit(robot,time);
    cells_.clear();
    if(!ready(time))return;
    for(const auto&v:samples_) {
      if(std::abs(v.second.point.z()-height(v.second.point))<=height_tolerance)cells_[v.first]=v.second;
    }
  }
  bool ready(double time) const {
    return plane_ && std::isfinite(time) && time-plane_->time>=-.15 && time-plane_->time<=max_age;
  }
  std::optional<double> height_at(const Eigen::Vector3d&p,double time) const {
    if(!p.allFinite()||!ready(time))return {};
    return height(p);
  }
  size_t size() const{return cells_.size();}
  size_t inliers() const{return plane_?plane_->inliers:0;}
  bool supports(const Eigen::Vector3d& p,double time) const {
    if(!p.allFinite()||!ready(time)||std::abs(p.z()-robot_.z())>height_tolerance)return false;
    const int x=int(std::floor(p.x()/resolution)),y=int(std::floor(p.y()/resolution)),n=int(std::ceil(radius/resolution));
    for(int j=-n;j<=n;++j)for(int i=-n;i<=n;++i) {
      auto it=cells_.find(key(x+i,y+j));if(it==cells_.end())continue;
      const auto&v=it->second;
      if(time-v.time>=-.15&&time-v.time<=max_age && (v.point-p).head<2>().norm()<=radius &&
         std::abs(v.point.z()-height(v.point))<=height_tolerance)return true;
    }
    return false;
  }
 private:
  struct Cell{Eigen::Vector3d point;double time;};
  struct Plane{Eigen::Vector3d origin,coefficients;double time;size_t inliers;};
  static uint64_t key(int x,int y){return (uint64_t(uint32_t(x))<<32)|uint32_t(y);}
  double height(const Eigen::Vector3d&p)const {
    const auto d=p-plane_->origin;
    return plane_->origin.z()+plane_->coefficients.x()*d.x()+plane_->coefficients.y()*d.y()+plane_->coefficients.z();
  }
  void fit(const Eigen::Vector3d&robot,double time) {
    std::vector<Cell> points;
    for(const auto&v:samples_)if((v.second.point-robot).head<2>().norm()<=fit_radius)points.push_back(v.second);
    if(points.size()<static_cast<size_t>(minimum_inliers))return;
    std::sort(points.begin(),points.end(),[](const Cell&a,const Cell&b){
      return a.point.x()<b.point.x()||(a.point.x()==b.point.x()&&a.point.y()<b.point.y());});
    // Bound the per-scan RANSAC cost even in dense outdoor scans.
    if(points.size()>600) {
      std::vector<Cell> reduced;
      for(size_t i=0;i<600;++i)reduced.push_back(points[i*points.size()/600]);
      points=std::move(reduced);
    }
    const double slope=std::tan(max_slope_degrees*3.141592653589793/180);
    auto plausible=[&](const Eigen::Vector3d&c){
      // Broad origin-relative search bounds, never an assumed floor band.
      // Reject overhead planes and near-vertical walls; allow both base and IMU origins.
      return c.allFinite()&&c.head<2>().norm()<=slope&&c.z()>=-1.0&&c.z()<=.15;
    };
    std::mt19937 random(1837);std::uniform_int_distribution<size_t> pick(0,points.size()-1);
    std::vector<size_t> best;
    for(int trial=0;trial<160;++trial) {
      const Eigen::Vector3d a=points[pick(random)].point-robot,b=points[pick(random)].point-robot,c=points[pick(random)].point-robot;
      const Eigen::Vector3d normal=(b-a).cross(c-a);
      if(std::abs(normal.z())<1e-4)continue;
      Eigen::Vector3d model(-normal.x()/normal.z(),-normal.y()/normal.z(),0);
      model.z()=a.z()-model.x()*a.x()-model.y()*a.y();
      if(!plausible(model))continue;
      std::vector<size_t> inliers;
      for(size_t i=0;i<points.size();++i) {
        const auto p=points[i].point-robot;
        if(std::abs(p.z()-model.x()*p.x()-model.y()*p.y()-model.z())<=fit_tolerance)inliers.push_back(i);
      }
      if(inliers.size()>best.size())best=std::move(inliers);
    }
    if(best.size()<static_cast<size_t>(minimum_inliers)||best.size()<.2*points.size())return;
    Eigen::Matrix3d normal=Eigen::Matrix3d::Zero();Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
    Eigen::Vector2d mean=Eigen::Vector2d::Zero();Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
    std::vector<double> times;
    for(size_t i:best) {
      const Eigen::Vector3d p=points[i].point-robot,row(p.x(),p.y(),1);
      normal+=row*row.transpose();rhs+=row*p.z();mean+=p.head<2>();
      covariance+=p.head<2>()*p.head<2>().transpose();times.push_back(points[i].time);
    }
    mean/=best.size();covariance=covariance/best.size()-mean*mean.transpose();
    const double smallest=.5*(covariance.trace()-std::hypot(covariance(0,0)-covariance(1,1),2*covariance(0,1)));
    if(smallest<.01)return; // A line or a wall foot is not an observed floor patch.
    Eigen::Vector3d model=normal.ldlt().solve(rhs);
    if(!plausible(model))return;
    std::sort(times.begin(),times.end());const double measured_time=times[times.size()/2];
    if(time-measured_time>max_age)return;
    if(ready(time)) {
      const double old_height=height(robot)-robot.z();
      // Reject a sudden switch to a different horizontal surface.
      if(std::abs(model.z()-old_height)>.12)return;
      model.head<2>()=.7*plane_->coefficients.head<2>()+.3*model.head<2>();
      model.z()=.7*old_height+.3*model.z();
    }
    plane_=Plane{robot,model,measured_time,best.size()};
  }
  std::unordered_map<uint64_t,Cell> samples_,cells_;
  std::optional<Plane> plane_;
  Eigen::Vector3d robot_=Eigen::Vector3d::Zero();
  double last_time_=-1;
};
}
