#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace rover_explore {
struct CorridorSection { double x, y, width; };
struct CorridorProjection {
  double lateral=0, yaw=0, width=0, arc=0, distance=1e30;
};
// Cache segment geometry once per controller cycle, not once per sampled pose.
class CorridorGeometry {
  struct Segment {double x,y,dx,dy,len,arc,width,slope,yaw,lo,hi;};
  std::vector<Segment> segments_;
public:
  explicit CorridorGeometry(const std::vector<CorridorSection>& line,
      double rear_extension=.95,double front_extension=.2) {
    double arc=0;
    for(size_t i=1;i<line.size();++i) {
      const auto&a=line[i-1];const auto&b=line[i];
      const double dx=b.x-a.x,dy=b.y-a.y,len=std::hypot(dx,dy);
      if(len<1e-5)continue;
      const auto&before=line[i>2?i-2:0];const auto&after=line[std::min(line.size()-1,i+1)];
      const double tangent=std::atan2(after.y-before.y,after.x-before.x);
      segments_.push_back({a.x,a.y,dx/len,dy/len,len,arc,a.width,
          (b.width-a.width)/len,tangent,
          i==1?-rear_extension:0.,len+(i+1==line.size()?front_extension:0.)});
      arc+=len;
    }
  }
  CorridorProjection project(double x,double y) const {
    CorridorProjection best;double best_squared=1e60;
    for(const auto&s:segments_) {
      const double rx=x-s.x,ry=y-s.y;
      const double t=std::clamp(rx*s.dx+ry*s.dy,s.lo,s.hi);
      const double ex=rx-t*s.dx,ey=ry-t*s.dy,d2=ex*ex+ey*ey;
      if(d2<best_squared) {
        best_squared=d2;
        best={s.dx*ry-s.dy*rx,s.yaw,s.width+std::clamp(t,0.,s.len)*s.slope,
            s.arc+t,0};
      }
    }
    best.distance=std::sqrt(best_squared);return best;
  }
  // Corners and edge midpoints use their own local widths on curved passages.
  template<class Points>
  double clearance(const Points& footprint,double x,double y,double yaw) const {
    if(segments_.empty()||footprint.size()<3)return -1e6;
    double clearance=1e6;const double c=std::cos(yaw),s=std::sin(yaw);
    for(size_t i=0;i<footprint.size();++i) {
      const auto&a=footprint[i];const auto&b=footprint[(i+1)%footprint.size()];
      for(double t:{0.,.5}) {
        const double bx=a.x+t*(b.x-a.x),by=a.y+t*(b.y-a.y);
        const auto p=project(x+c*bx-s*by,y+s*bx+c*by);
        clearance=std::min(clearance,.5*p.width-p.distance);
      }
    }
    return clearance;
  }
};
inline CorridorProjection project_corridor(const std::vector<CorridorSection>& line,
    double x,double y) {return CorridorGeometry(line).project(x,y);}
template<class Points>
inline double corridor_clearance(const std::vector<CorridorSection>& line,
    const Points& footprint,double x,double y,double yaw) {
  return CorridorGeometry(line).clearance(footprint,x,y,yaw);
}
} // namespace rover_explore
