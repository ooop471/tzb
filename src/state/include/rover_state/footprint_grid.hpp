#pragma once
#include "rover_state/geometry.hpp"
#include <algorithm>
#include <cstdint>

namespace rover_state {
// Canonical costs: 0 traversable, 1 inscribed, 2 lethal, 3 unknown.
// Check the centre against inscribed costs, the physical rectangle against lethal
// costs, so the footprint does not receive the costmap's inflation twice.
struct FootprintGrid {
  double resolution{};int width{},height{};Pose origin;std::vector<uint8_t> cells;
  Point coordinates(Point p) const {
    const double dx=p.x-origin.p.x,dy=p.y-origin.p.y,c=std::cos(origin.yaw),s=std::sin(origin.yaw);
    return {(c*dx+s*dy)/resolution,(-s*dx+c*dy)/resolution,0};
  }
  bool contains(Point p) const {const auto q=coordinates(p);return q.x>=0&&q.y>=0&&q.x<width&&q.y<height;}
  bool centre_free(Point p,bool required) const {
    if(!contains(p))return !required;
    const auto q=coordinates(p);return cells[static_cast<int>(q.y)*width+static_cast<int>(q.x)]==0;
  }
  bool rectangle_free(Pose pose,double front,double rear,double half_width,double margin,bool required) const {
    const double yaw=pose.yaw-origin.yaw,c=std::cos(yaw),s=std::sin(yaw);
    const double hx=(front+rear)*.5/resolution,hy=half_width/resolution;
    Point centre=pose.p;centre.x+=(front-rear)*.5*std::cos(pose.yaw);centre.y+=(front-rear)*.5*std::sin(pose.yaw);
    centre=coordinates(centre);const double a=hx+margin/resolution,b=hy+margin/resolution;
    const double ex=a*std::abs(c)+b*std::abs(s),ey=a*std::abs(s)+b*std::abs(c);
    const int x0=static_cast<int>(std::floor(centre.x-ex)),x1=static_cast<int>(std::floor(centre.x+ex));
    const int y0=static_cast<int>(std::floor(centre.y-ey)),y1=static_cast<int>(std::floor(centre.y+ey));
    if(required && (x0<0||y0<0||x1>=width||y1>=height))return false;
    for(int y=std::max(0,y0);y<=std::min(height-1,y1);++y)
      for(int x=std::max(0,x0);x<=std::min(width-1,x1);++x) {
        if(cells[y*width+x]<2)continue;
        const double dx=x+.5-centre.x,dy=y+.5-centre.y;
        if(std::abs(dx)>ex+.5||std::abs(dy)>ey+.5)continue;
        if(std::abs(c*dx+s*dy)>a+.5*(std::abs(c)+std::abs(s)))continue;
        if(std::abs(-s*dx+c*dy)>b+.5*(std::abs(c)+std::abs(s)))continue;
        return false;
      }
    return true;
  }
};
} // namespace rover_state
