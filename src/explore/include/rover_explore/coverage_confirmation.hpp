// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <optional>
#include <stdexcept>

namespace rover_explore
{
// Count fresh planning updates, not timer ticks. Completion is reversible.
class CoverageConfirmation
{
public:
  CoverageConfirmation(double duration = 10.0, int cycles = 10, double max_gap = 2.0)
      : duration_(duration), max_gap_(max_gap), cycles_(cycles)
  {
    if (!std::isfinite(duration) || duration <= 0 || cycles < 2 ||
        !std::isfinite(max_gap) || max_gap <= 0)
      throw std::invalid_argument("Invalid coverage confirmation interval/count");
  }
  void reset() { since_.reset(); last_.reset(); count_ = 0; }
  bool update(bool no_work, double stamp)
  {
    if (!no_work || !std::isfinite(stamp)) { reset(); return false; }
    if (!last_ || stamp <= *last_ || stamp - *last_ > max_gap_)
    {
      reset();
      since_ = stamp;
    }
    last_ = stamp;
    ++count_;
    return count_ >= cycles_ && stamp - *since_ >= duration_;
  }
private:
  double duration_, max_gap_;
  int cycles_, count_{0};
  std::optional<double> since_, last_;
};
} // namespace rover_explore
