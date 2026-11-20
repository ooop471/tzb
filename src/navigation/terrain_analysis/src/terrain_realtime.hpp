#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace terrain_realtime {
// Same order statistic as sort()[index], without sorting every elevation.
inline float ground_quantile(std::vector<float>& elevations, double fraction,
                             bool limit_lift, float max_lift) {
  const auto index = std::min(elevations.size()-1,
      static_cast<std::size_t>(std::max(0., fraction) * elevations.size()));
  const float minimum = limit_lift ? *std::min_element(elevations.begin(), elevations.end()) : 0.f;
  std::nth_element(elevations.begin(), elevations.begin()+index, elevations.end());
  return limit_lift ? std::min(elevations[index], minimum+max_lift) : elevations[index];
}
struct Cell {
  int64_t x, y, z;
  bool operator==(const Cell& other) const { return x==other.x && y==other.y && z==other.z; }
};
struct CellHash {
  std::size_t operator()(const Cell& cell) const {
    std::size_t h=std::hash<int64_t>{}(cell.x);
    for (auto value : {cell.y,cell.z}) h ^= std::hash<int64_t>{}(value)+0x9e3779b9+(h<<6)+(h>>2);
    return h;
  }
};
// Keep an actual obstacle return, never an averaged point/height. Choosing the
// greatest ground-relative height preserves thin/tall returns within each cell.
// floor(), rather than truncation, keeps negative-world-coordinate cells distinct.
template<class Point>
class ObstacleUnion {
 public:
  explicit ObstacleUnion(double leaf) : leaf_(leaf) {}
  void clear() { indices_.clear(); points_.clear(); }
  template<class Range> void add(const Range& points) {
    add(points, [](const Point&) { return true; });
  }
  template<class Range, class Predicate> void add(const Range& points, Predicate keep) {
    for (const auto& p : points) {
      if(!keep(p))continue;
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity))continue;
      const Cell cell{static_cast<int64_t>(std::floor(p.x/leaf_)),
                      static_cast<int64_t>(std::floor(p.y/leaf_)),
                      static_cast<int64_t>(std::floor(p.z/leaf_))};
      const auto inserted=indices_.emplace(cell,points_.size());
      if(inserted.second)points_.push_back(p);
      else if(p.intensity>points_[inserted.first->second].intensity)points_[inserted.first->second]=p;
    }
  }
  const std::vector<Point>& points() const { return points_; }
 private:
  double leaf_;
  std::unordered_map<Cell,std::size_t,CellHash> indices_;
  std::vector<Point> points_;
};
}  // namespace terrain_realtime
