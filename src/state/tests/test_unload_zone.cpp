#include "rover_state/unload_zone.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace rover_state;
int main() {
  UnloadZone z;z.validate();Pose zone{{12.3,-2.3,0.},0.};
  // The chassis centre at the destination leaves the bin 40 cm short.
  assert(!z.contains(zone,zone));
  for(double yaw:{0.,.4,1.57,-1.2,3.14}) {
    const auto base=z.base_goal(zone,yaw);const auto bin=z.reference(base);
    assert(distance(bin,zone.p)<1e-12);assert(z.contains(base,zone));
  }
  Pose rotated=zone;rotated.yaw=.7;
  assert(z.contains_point({zone.p.x+.15,zone.p.y,0.},rotated));
  assert(!z.contains_point({zone.p.x+.30,zone.p.y,0.},zone));
  assert(!z.contains_point({zone.p.x+.21,zone.p.y,0.},zone)); // 5 cm edge reserve
  assert(!z.contains_point({std::numeric_limits<double>::quiet_NaN(),0.,0.},zone));
  // Reproduce the last run: the BASE was near the goal, the BIN was outside.
  const Pose previous{{12.154,-2.025,0.},-.133};
  const Pose actual_zone{{12.287,-2.278,0.},0.};
  assert(!z.contains(previous,actual_zone));
  assert(z.contains(z.base_goal(actual_zone,previous.yaw),actual_zone));
  z.margin=.25;bool rejected=false;try{z.validate();}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  std::cout<<"PASS: calibrated bin point, arbitrary heading, square margin, rotated zone and previous-run regression\n";
}
