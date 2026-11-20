#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <unordered_set>

namespace rover_state {
// Steady wall time includes pauses and cannot be extended by ROS clock changes.
class MissionBudget {
 public:
  using Clock=std::chrono::steady_clock;
  using Time=Clock::time_point;
  double total_seconds{840.}, grasp_seconds{600.};
  int max_grasps{10};
  void reset() {start_.reset();finish_.reset();counted_.clear();closed_=false;}
  void start(Time now) {reset();start_=now;}
  void finish(Time now) {if(start_&&!finish_)finish_=now;}
  double elapsed(Time now) const {return start_?std::max(0.,std::chrono::duration<double>((finish_?*finish_:now)-*start_).count()):0.;}
  double remaining(Time now) const {return std::max(0.,total_seconds-elapsed(now));}
  void completed(uint64_t id) {if(start_)counted_.insert(id);}
  int count() const {return static_cast<int>(counted_.size());}
  bool update(Time now) {closed_ |= start_ && (count()>=max_grasps || elapsed(now)>=grasp_seconds);return closed_;}
  bool closed() const {return closed_;}
 private:
  std::optional<Time> start_,finish_;
  std::unordered_set<uint64_t> counted_;
  bool closed_{false};
};
}
