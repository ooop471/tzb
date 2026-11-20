#include "rover_state/mission_budget.hpp"
#include <cassert>
#include <iostream>
using namespace rover_state;
int main() {
  MissionBudget b;const MissionBudget::Time t{};
  auto at=[&](double s){return t+std::chrono::duration_cast<MissionBudget::Clock::duration>(std::chrono::duration<double>(s));};
  assert(!b.update(at(1000))); // idle does not consume the budget
  b.start(t);assert(!b.update(at(599.999)));
  assert(b.update(at(600.)));assert(b.remaining(at(600.))==240.);
  assert(b.update(at(599.))); // closing is latched, even with an older sample
  b.start(t);b.completed(7);b.completed(7);assert(b.count()==1);
  for(uint64_t id=8;id<16;++id) {b.completed(id);assert(!b.update(at(40.)));}
  assert(b.count()==9);
  b.completed(16);assert(b.update(at(40.)));assert(b.count()==10);
  assert(b.elapsed(at(500.))==500.); // no pause/resume operation can reset time
  assert(b.remaining(at(850.))==0.);
  b.finish(at(550.));assert(b.elapsed(at(700.))==550.);
  b.reset();assert(!b.closed()&&b.count()==0&&b.elapsed(at(900.))==0.);
  std::cout<<"PASS: 600 s cutoff, 10 successes, duplicate result, pause time, reset, 840 s total\n";
}
