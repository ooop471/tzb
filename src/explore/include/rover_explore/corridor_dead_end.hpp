#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace rover_explore {
// Physical map evidence, deliberately separate from inflated navigation costs.
enum class PhysicalCell { Unknown, Free, Obstacle };
struct CorridorEndObservation {
  bool closed=false;
  Eigen::Vector3d position=Eigen::Vector3d::Zero();
  double yaw=0, width=0;
  std::string reason="UNKNOWN";
};

// A positive U-shaped enclosure test in corridor coordinates. Missing walls,
// missing ground and failed planning are never evidence of a dead end.
class CorridorDeadEnd {
public:
  using Point=Eigen::Vector3d;
  using Reader=std::function<PhysicalCell(const Point&)>;
  using Supported=std::function<bool(const Point&)>;
  static CorridorEndObservation detect(const Point &origin,double yaw,double width,
      double maximum_distance,double passage_width,const std::vector<Point>& walls,
      const Reader &cell,const Supported &supported) {
    CorridorEndObservation result;
    if(!origin.allFinite()||!std::isfinite(yaw)||!std::isfinite(width)||width<passage_width||
       !std::isfinite(maximum_distance)||maximum_distance<.5||!cell||!supported) return result;
    const Point forward(std::cos(yaw),std::sin(yaw),0),left(-std::sin(yaw),std::cos(yaw),0);
    std::vector<Eigen::Vector2d> points;
    for(const auto &p:walls) if(p.allFinite()) {
      const Point d=p-origin;
      if(std::abs(d.dot(left))<=width*.5+.3 && d.dot(forward)>=-.2 &&
         d.dot(forward)<=maximum_distance+.2) points.emplace_back(d.dot(forward),d.dot(left));
    }
    const auto at=[&](double x,double y)->Point{return origin+x*forward+y*left;};
    // Both current low-wall returns and lethal cells must support each strip.
    // Occupancy inflation, unknown cells and synthetic boundary caps cannot pass.
    const auto wall=[&](double x,double y,double dx,double dy) {
      bool observed=false;
      for(const auto &p:points) if(std::abs(p.x()-x)<=dx&&std::abs(p.y()-y)<=dy) {observed=true;break;}
      if(!observed)return false;
      for(double a=-dx;a<=dx+1e-6;a+=.04)
        for(double b=-dy;b<=dy+1e-6;b+=.04)
          if(cell(at(x+a,y+b))==PhysicalCell::Obstacle)return true;
      return false;
    };
    for(double distance=.5;distance<=maximum_distance+1e-6;distance+=.05) {
      const int bins=std::max(8,static_cast<int>(std::ceil(width/.10)));
      int hits=0,missing=0,longest_gap=0;
      bool ends=true;
      for(int i=0;i<=bins;++i) {
        const bool hit=wall(distance,-width*.5+width*i/bins,.12,.065);
        if(hit){++hits;missing=0;}else{longest_gap=std::max(longest_gap,++missing);}
        if((i==0||i==bins)&&!hit)ends=false;
      }
      if(!ends||hits<.85*(bins+1)||longest_gap*width/bins>.15)continue;

      // Check for a connected lateral continuation independently of whether the
      // limited-angle centerline fitter can produce a turning path.
      bool opening=false;
      for(int side:{-1,1}) {
        for(double x=std::max(.1,distance-width);x<=distance-.1;x+=.1) {
          bool clear=true;
          for(double a=-passage_width*.5;a<=passage_width*.5+1e-6&&clear;a+=.04)
            for(double b=std::max(0.,width*.5-.25);b<=width*.5+.35;b+=.02) {
              const Point p=at(x+a,side*b);
              if(cell(p)!=PhysicalCell::Free||!supported(p)){clear=false;break;}
            }
          if(clear){opening=true;break;}
        }
      }
      if(opening){result.reason="SIDE_CONTINUATION";return result;}

      // Require both observed side walls right up to the cap. A 90-degree bend
      // loses its inner wall here; occlusion is UNKNOWN rather than CLOSED.
      bool enclosed=true;
      const double start=std::max(.05,distance-std::max(width,.7));
      if(distance-.15-start<.3)continue;
      for(int side:{-1,1})
        for(double x=start;x<=distance-.15+1e-6;x+=.1)
          if(!wall(x,side*width*.5,.08,.14))enclosed=false;
      // Positive free/ground evidence in the approach keeps an arm-sized return
      // or an entirely unobserved region from being treated as a closed tunnel.
      bool ground_seen=false;
      for(double x=std::max(.65,start);x<=distance-.25;x+=.15) {
        ground_seen=true;
        const Point p=at(x,0);
        if(cell(p)!=PhysicalCell::Free||!supported(p))enclosed=false;
      }
      if(!enclosed||!ground_seen)continue;
      result.closed=true;result.position=at(distance,0);result.yaw=yaw;result.width=width;
      result.reason="OBSERVED_WALL_ENCLOSURE";return result;
    }
    return result;
  }
};
} // namespace rover_explore
