#include "rover_explore/corridor_centerline.hpp"
#include <cassert>
#include <iostream>
using namespace rover_explore;
using Point=Eigen::Vector3d;

struct Scene {
  double width=1.2,cap=1.8,yaw=0;
  Point origin=Point::Zero();
  bool left=true,right=true,end=true,bend=false,unknown=false,map_obstacles=true,returns=true,ground=true;
  double cap_gap=0;
  Point world(double x,double y)const {
    return origin+Point(std::cos(yaw)*x-std::sin(yaw)*y,std::sin(yaw)*x+std::cos(yaw)*y,0);
  }
  Point local(const Point&p)const {
    const auto d=p-origin;return {std::cos(yaw)*d.x()+std::sin(yaw)*d.y(),
      -std::sin(yaw)*d.x()+std::cos(yaw)*d.y(),0};
  }
  bool obstacle(double x,double y)const {
    return (left&&std::abs(y-width*.5)<.025&&x>=0&&x<=cap+.05)||
      (right&&std::abs(y+width*.5)<.025&&x>=0&&x<=(bend?cap-width:cap+.05))||
      (end&&std::abs(x-cap)<.025&&std::abs(y)<=width*.5+.025&&std::abs(y)>=cap_gap*.5);
  }
  CorridorEndObservation detect()const {
    std::vector<Point> walls;
    if(returns) {
      for(double x=0;x<=cap+.02;x+=.02)for(int side:{-1,1})
        if(obstacle(x,side*width*.5))walls.push_back(world(x,side*width*.5));
      for(double y=-width*.5;y<=width*.5+.001;y+=.02)
        if(obstacle(cap,y))walls.push_back(world(cap,y));
    }
    return CorridorDeadEnd::detect(origin,yaw,width,2.,.7,walls,
      [&](const Point&p){const auto q=local(p);
        if(unknown&&q.x()>cap-.2)return PhysicalCell::Unknown;
        return map_obstacles&&obstacle(q.x(),q.y())?PhysicalCell::Obstacle:PhysicalCell::Free;},
      [&](const Point&){return ground;});
  }
};

int main() {
  Scene s;auto end=s.detect();assert(end.closed);assert((end.position-s.world(s.cap,0)).norm()<.15);
  // Corridor frame remains valid at a bend or while the chassis faces elsewhere.
  s.yaw=1.7;s.origin={4.,-2.,0.};assert(s.detect().closed);
  s.bend=true;assert(!s.detect().closed); // 90-degree right continuation
  s.bend=false;s.left=false;assert(!s.detect().closed); // T-junction/one wall missing
  s.left=true;s.end=false;assert(!s.detect().closed); // open straight corridor
  s.end=true;s.unknown=true;assert(!s.detect().closed); // map occlusion
  s.unknown=false;s.map_obstacles=false;assert(!s.detect().closed); // inflation/returns alone
  s.map_obstacles=true;s.returns=false;assert(!s.detect().closed); // stale map alone
  s.returns=true;s.ground=false;assert(!s.detect().closed); // unobserved floor
  s.ground=true;
  s.returns=true;s.cap_gap=.75;assert(!s.detect().closed); // usable passage in cap
  s.cap_gap=0;s.cap=2.8;assert(!s.detect().closed); // beyond the local detection horizon
  // Exercise the production wall-pair fitter -> detector integration as well.
  s=Scene{};
  CorridorCenterline measured;
  measured.parameters.enabled=true;
  std::vector<Point> measured_walls;
  for(double x=0;x<=s.cap;x+=.02)for(double y:{-.6,.6})measured_walls.push_back(s.world(x,y));
  for(double y=-.6;y<=.6;y+=.02)measured_walls.push_back(s.world(s.cap,y));
  const auto physical=[&](const Point&p) {
    const auto q=s.local(p);return s.obstacle(q.x(),q.y())?PhysicalCell::Obstacle:PhysicalCell::Free;
  };
  measured.update(s.origin,s.yaw,1.,measured_walls,{},[](const Point&){return true;},
      [](const Point&){return true;},[](const Point&)->std::optional<double>{return -.2;},physical);
  assert(measured.end_observation().closed);
  measured.update(s.origin,s.yaw+.4,1.2,measured_walls,{},[](const Point&){return true;},
      [](const Point&){return true;},[](const Point&)->std::optional<double>{return -.2;},physical);
  assert(measured.end_observation().closed); // body heading does not define the endpoint
  // An explicit post-turn reset changes direction; a grasp pause does not.
  CorridorCenterline corridor;corridor.parameters.enabled=true;
  corridor.reorient(Point::Zero(),3.141592653589793);
  std::vector<Point> walls;
  for(double x=-.05;x>=-3.;x-=.04)for(double y:{-.55,.55})walls.emplace_back(x,y,0);
  auto target=corridor.update(Point::Zero(),3.141592653589793,1.,walls,{},[](const Point&){return true;});
  target=corridor.update(Point::Zero(),3.141592653589793,1.2,walls,{},[](const Point&){return true;});
  assert(target&&target->x()<0);
  corridor.release_target();
  target=corridor.update(Point::Zero(),0.,1.4,walls,{},[](const Point&){return true;});
  assert(target&&target->x()<0);
  std::cout<<"PASS: physical corridor closure, bends, missing walls, unknown maps, gaps, range and explicit reorientation\n";
}
