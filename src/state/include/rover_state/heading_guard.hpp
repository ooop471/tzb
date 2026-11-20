#pragma once
#include <algorithm>

namespace rover_state {
// Restrict outward turning near the corridor heading bound; when already
// outside it, permit correction back toward the corridor instead of deadlock.
inline double corridor_angular_command(double command,double error,double bound,double rate) {
  constexpr double horizon=.10; // two 50 ms gate cycles
  double lower=std::clamp((-bound-error)/horizon,-rate,rate);
  double upper=std::clamp((bound-error)/horizon,-rate,rate);
  if(error>bound)upper=std::min(upper,-std::min(rate,.18));
  if(error< -bound)lower=std::max(lower,std::min(rate,.18));
  return std::clamp(command,lower,upper);
}
} // namespace rover_state
