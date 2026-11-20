#pragma once
#include "rover_explore/corridor_geometry.hpp"
#include <stdexcept>

namespace rover_explore {
struct CenterPose {double x,y,yaw;};
// Build a short, smooth joining path. The lateral quintic matches the initial
// heading and converges to zero offset and zero offset derivative on the center.
inline std::vector<CenterPose> centerline_path(const std::vector<CorridorSection>& line,
    double x,double y,double yaw,double target_x,double target_y,double step=.05) {
  if(line.size()<2||!std::isfinite(step)||step<=0)throw std::runtime_error("missing centerline");
  std::vector<double> arc{0};
  for(size_t i=1;i<line.size();++i) {
    const double length=std::hypot(line[i].x-line[i-1].x,line[i].y-line[i-1].y);
    if(!std::isfinite(length)||length<1e-5)throw std::runtime_error("invalid centerline segment");
    arc.push_back(arc.back()+length);
  }
  const CorridorGeometry geometry(line);
  const auto origin=geometry.project(x,y),end=geometry.project(target_x,target_y);
  const double length=end.arc-origin.arc;
  if(length<.19||length>2.1||origin.distance>.8||end.distance>.2)
    throw std::runtime_error("target or robot outside supported centerline range");
  const double angle=std::atan2(std::sin(yaw-origin.yaw),std::cos(yaw-origin.yaw));
  if(std::abs(angle)>1.2)throw std::runtime_error("body heading too far from centerline for a forward join");
  const auto sample=[&](double station) {
    const size_t i=std::min(line.size()-2,station<=0?size_t(0):
      size_t(std::upper_bound(arc.begin(),arc.end(),station)-arc.begin()-1));
    const double len=arc[i+1]-arc[i],u=(station-arc[i])/len;
    if(u<0||u>1)return CenterPose{line[i].x+u*(line[i+1].x-line[i].x),
      line[i].y+u*(line[i+1].y-line[i].y),std::atan2(line[i+1].y-line[i].y,line[i+1].x-line[i].x)};
    const auto derivative=[&](size_t k,bool y_axis){
      const size_t a=k==0?0:k-1,b=std::min(k+1,line.size()-1);
      return ((y_axis?line[b].y:line[b].x)-(y_axis?line[a].y:line[a].x))/(arc[b]-arc[a]);
    };
    const double h00=2*u*u*u-3*u*u+1,h10=u*u*u-2*u*u+u;
    const double h01=-2*u*u*u+3*u*u,h11=u*u*u-u*u;
    const double dh00=(6*u*u-6*u)/len,dh10=3*u*u-4*u+1;
    const double dh01=(-6*u*u+6*u)/len,dh11=3*u*u-2*u;
    const double vx=dh00*line[i].x+dh10*derivative(i,false)+dh01*line[i+1].x+dh11*derivative(i+1,false);
    const double vy=dh00*line[i].y+dh10*derivative(i,true)+dh01*line[i+1].y+dh11*derivative(i+1,true);
    return CenterPose{h00*line[i].x+h10*len*derivative(i,false)+h01*line[i+1].x+h11*len*derivative(i+1,false),
      h00*line[i].y+h10*len*derivative(i,true)+h01*line[i+1].y+h11*len*derivative(i+1,true),std::atan2(vy,vx)};
  };
  const double merge=std::min(1.5,length),slope=std::tan(angle)*merge;
  const int count=std::max(2,static_cast<int>(std::ceil(length/step)));
  if(count>2000)throw std::runtime_error("centerline path too long");
  std::vector<CenterPose> path;
  for(int i=0;i<=count;++i) {
    const double advance=length*i/count,u=std::clamp(advance/merge,0.,1.);
    const double u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
    const double offset=origin.lateral*(1-10*u3+15*u4-6*u5)+slope*(u-6*u3+8*u4-3*u5);
    auto p=sample(origin.arc+advance);p.x-=offset*std::sin(p.yaw);p.y+=offset*std::cos(p.yaw);path.push_back(p);
  }
  path.front()={x,y,yaw};
  for(size_t i=1;i<path.size();++i) {
    const auto&a=path[i-1];const auto&b=path[std::min(i+1,path.size()-1)];
    path[i].yaw=std::atan2(b.y-a.y,b.x-a.x);
    if(!std::isfinite(path[i].x)||!std::isfinite(path[i].y)||!std::isfinite(path[i].yaw))
      throw std::runtime_error("nonfinite centerline path");
  }
  return path;
}
}
