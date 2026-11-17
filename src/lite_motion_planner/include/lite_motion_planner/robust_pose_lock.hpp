#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
namespace lite_motion_planner {
inline bool grasp_height_acceptable(double visual,double reference,double deviation,bool use_visual) {
  return std::isfinite(visual)&&(use_visual||!std::isfinite(reference)||std::abs(visual-reference)<=deviation);
}
struct PoseLockConsensus {
  std::array<double,3> center{};
  std::vector<size_t> inliers;
  double max_deviation=0.;
  bool stable=false;
};
inline PoseLockConsensus pose_lock_consensus(const std::vector<std::array<double,3>>& points,
    double radius,size_t minimum=5) {
  PoseLockConsensus out;if(points.size()<minimum)return out;
  const auto median=[](std::vector<double> v){std::sort(v.begin(),v.end());return v[v.size()/2];};
  for(size_t k=0;k<3;++k){std::vector<double> v;for(const auto&p:points)v.push_back(p[k]);out.center[k]=median(v);}
  for(size_t i=0;i<points.size();++i){double d=0.;for(size_t k=0;k<3;++k)d+=std::pow(points[i][k]-out.center[k],2);if(std::isfinite(d)&&(radius<=0||d<=radius*radius))out.inliers.push_back(i);}
  if(out.inliers.size()<minimum||out.inliers.size()*5<points.size()*4)return out;
  for(size_t k=0;k<3;++k){std::vector<double> v;for(auto i:out.inliers)v.push_back(points[i][k]);out.center[k]=median(v);}
  for(auto i:out.inliers){double d=0.;for(size_t k=0;k<3;++k)d+=std::pow(points[i][k]-out.center[k],2);out.max_deviation=std::max(out.max_deviation,std::sqrt(d));}
  out.stable=radius<=0||out.max_deviation<=radius;return out;
}
}
