#include "../src/terrain_realtime.hpp"
#include <iostream>
#include <stdexcept>
struct Point { float x,y,z,intensity; };
void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
int main() {
  for(int count=1;count<=300;++count) {
    std::vector<float> sorted;
    for(int i=0;i<count;++i)sorted.push_back(float((i*53+count*11)%101)/17.f);
    std::sort(sorted.begin(),sorted.end());
    for(double q : {0.,.25,.5,.99,1.}) {
      auto v=sorted;std::reverse(v.begin(),v.end());
      const float reference=sorted[std::min(sorted.size()-1,size_t(q*sorted.size()))];
      check(terrain_realtime::ground_quantile(v,q,false,.15f)==reference,"quantile changed");
      check(terrain_realtime::ground_quantile(v,q,true,.15f)==std::min(reference,sorted[0]+.15f),"lift cap changed");
    }
  }
  terrain_realtime::ObstacleUnion<Point> cloud(.05);
  std::vector<Point> points{{.011f,.011f,.311f,.2f},{.019f,.019f,.319f,.4f},
                            {-.011f,.011f,.311f,.3f},{.011f,.011f,.371f,.5f}};
  for(int i=0;i<9;++i)cloud.add(points);
  check(cloud.points().size()==3,"duplicate or negative-coordinate cell handling changed");
  check(cloud.points()[0].x==points[1].x && cloud.points()[0].intensity==.4f,"strongest real obstacle return lost/averaged");
  check(cloud.points()[1].x<0 && cloud.points()[2].z>.35f,"thin obstacle cells lost");
  cloud.clear();check(cloud.points().empty(),"clear retained obstacles");
  cloud.add(points,[](const Point& p){return p.x<.015f;});
  check(cloud.points()[0].x==points[0].x,"filtered high return hid a valid cell representative");
  std::cout<<"Quantile equivalence, lift cap, signed cells, obstacle height and deduplication passed\n";
}
