#include "rover_explore/ceiling_exit.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
using Detector=rover_explore::CeilingExit;using Point=Detector::Point;
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
std::vector<Point> roof(double height,Point origin=Point::Zero()) {
  std::vector<Point> points;
  for(int x=0;x<15;++x)for(int y=0;y<15;++y)points.push_back(origin+Point(.1+.05*x,-.35+.05*y,height));
  return points;
}
void update(Detector& d,const std::vector<Point>& p,double time,Point origin=Point::Zero()){
  // No ground estimate is available: exit height must not depend on it.
  d.update(p,origin,0.,time,[](const Point&)->std::optional<double>{return {};},origin);
}
void hold(Detector& d,const std::vector<Point>& p,double begin=1.,Point origin=Point::Zero()){
  for(int i=0;i<7;++i)update(d,p,begin+i*.2,origin);
}
int main(){
  {Detector d;d.parameters.validate();hold(d,roof(2.01));require(bool(d.detection()),"all-high visible upper returns did not trigger");}
  for(double height:{1.8,1.999,2.}) {
    Detector d;d.parameters.minimum_high_fraction=1.;auto p=roof(3.);p.push_back(Point(.3,0.,height));hold(d,p);
    require(d.statistics.low_points==1&&!d.detection(),"a single <=2m point did not veto exit");
  }
  {Detector d;hold(d,{});require(!d.detection()&&!d.statistics.enough_returns,"empty sky counted as all high");
   hold(d,{Point(.2,0.,3.)},3.);require(!d.detection(),"sparse returns triggered exit");}
  {Detector d;auto p=roof(3.);update(d,p,1.);update(d,p,1.2);require(!d.detection(),"no temporal confirmation");
   auto low=roof(1.9);update(d,low,1.4);
   update(d,p,1.6);update(d,p,1.8);require(!d.detection(),"veto failed to reset confirmation");
   update(d,{},2.);update(d,p,2.2);update(d,p,2.4);require(!d.detection(),"missing scan failed to reset confirmation");
   hold(d,p,2.6);require(bool(d.detection()),"valid observations did not recover");}
  {Detector d;const auto p=roof(3.);update(d,p,1.);update(d,p,1.2);update(d,p,2.);update(d,p,2.2);
   require(!d.detection(),"scan gap retained old confirmation");}
  {Detector d;Point origin(7.,-2.,1.5);hold(d,roof(1.9,origin),1.,origin);
   require(!d.detection(),"world height was mistaken for height above lidar");
   hold(d,roof(2.1,origin),3.,origin);require(bool(d.detection()),"lidar-relative height was not used");}
  {Detector d;Point origin(4.,1.,.4);const auto correct=roof(2.1,origin);
   const Eigen::Matrix3d rotation=Eigen::AngleAxisd(-40.*std::acos(-1.)/180.,Point::UnitY()).toRotationMatrix();
   std::vector<Point> transformed;
   for(const auto& p:correct){const Point raw=rotation.transpose()*(p-origin);transformed.push_back(origin+rotation*raw);}
   hold(d,transformed,1.,origin);require(bool(d.detection()),"tilted raw returns lost vertical height after pose transform");}
  {Detector d;auto p=roof(2.1);p.push_back(Point(1.5,0.,.4));p.push_back(Point(.1,0.,.1));
   p.push_back(Point(4.,0.,1.));p.push_back(Point(.3,0.,20.));
   hold(d,p);require(bool(d.detection()),"wall, body or out-of-range return counted as overhead");}
  for(int high : {54,55,60}) {
    Detector d;auto p=roof(3.);p.resize(100);
    for(int i=high;i<100;++i)p[i].z()=2.;
    hold(d,p);
    require(bool(d.detection())==(high>=55),"55 percent boundary failed");
    require(d.statistics.low_points==100-high,"2m boundary classification changed");
  }
  {Detector d;auto p=roof(3.);p.push_back(Point(.3,0.,1.8));hold(d,p);
   require(bool(d.detection()),"one low return still vetoes majority policy");}
  {Detector d;auto high=roof(3.);update(d,high,1.);update(d,high,1.2);
   auto mixed=high;for(int i=0;i<120;++i)mixed[i].z()=1.9;
   update(d,mixed,1.4);update(d,high,1.6);update(d,high,1.8);
   require(!d.detection(),"below-ratio frame failed to reset confirmation");}
  for(double ratio : {0.,1.01}){Detector d;d.parameters.minimum_high_fraction=ratio;
    bool rejected=false;try{d.parameters.validate();}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"invalid ratio accepted");}
  std::cout<<"Relative-lidar upper returns: strict 2m boundary, configurable strict/majority policy, 54/55/60 percent boundaries, no-ground height, 40deg tilt, empty/sparse input, temporal reset and region filtering passed\n";
}
