#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace rover_explore {
class CorridorFitStability {
  std::optional<double> pending_shift_;
  int confirmations_=0;
public:
  void reset(){pending_shift_.reset();confirmations_=0;}
  template<class Section>
  std::vector<Section> update(std::vector<Section> fit,const std::vector<Section>& previous,
      double axis,double dt,double smoothing_time,double maximum_step,
      double jump_threshold,int required_confirmations) {
    if(previous.size()<2||fit.size()<2){reset();return fit;}
    const Eigen::Vector2d left(-std::sin(axis),std::cos(axis));
    const auto prior=[&](const Eigen::Vector3d &point)->std::optional<Eigen::Vector2d> {
      double best=1e30;std::optional<Eigen::Vector2d> foot;
      for(size_t i=1;i<previous.size();++i) {
        const Eigen::Vector2d a=previous[i-1].center.template head<2>();
        const Eigen::Vector2d delta=(previous[i].center-previous[i-1].center).template head<2>();
        const double length=delta.norm();if(length<1e-6)continue;
        const Eigen::Vector2d relative=point.head<2>()-a;
        const double t=std::clamp(relative.dot(delta)/(length*length),i==1?-.5/length:0.,i+1==previous.size()?1.+.5/length:1.);
        const Eigen::Vector2d q=a+t*delta;const double d=(point.head<2>()-q).norm();
        if(d<best){best=d;foot=q;}
      }
      return best<.8 ? foot : std::nullopt;
    };
    std::vector<double> shifts;
    for(const auto&s:fit)if(const auto p=prior(s.center))shifts.push_back((s.center.template head<2>()-*p).dot(left));
    if(shifts.size()<2){reset();return fit;}
    std::sort(shifts.begin(),shifts.end());const double shift=shifts[shifts.size()/2];
    if(std::abs(shift)>jump_threshold) {
      if(pending_shift_&&std::abs(shift-*pending_shift_)<jump_threshold*.5)++confirmations_;
      else confirmations_=1;
      pending_shift_=shift;
      if(confirmations_<required_confirmations)return previous;
    } else reset();
    const double alpha=std::clamp(dt/(smoothing_time+dt),0.,1.);
    for(auto &s:fit)if(const auto p=prior(s.center)) {
      const double lateral=(s.center.template head<2>()-*p).dot(left);
      const double accepted=std::clamp(alpha*lateral,-maximum_step,maximum_step);
      s.center.template head<2>()-=(lateral-accepted)*left;
    }
    return fit;
  }
};
} // namespace rover_explore
