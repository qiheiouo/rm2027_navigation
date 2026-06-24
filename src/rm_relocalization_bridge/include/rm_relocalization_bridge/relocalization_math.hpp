#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "tf2/LinearMath/Transform.h"

namespace rm_relocalization_bridge
{

struct TimedTransform
{
  std::int64_t stamp_nanoseconds;
  tf2::Transform transform;
};

class TimedTransformCache
{
public:
  explicit TimedTransformCache(std::size_t max_size);

  // Returns false when a non-monotonic timestamp resets the cache.
  bool add(const TimedTransform & sample);
  std::optional<TimedTransform> nearest(
    std::int64_t stamp_nanoseconds,
    std::int64_t tolerance_nanoseconds) const;
  void clear();
  std::size_t size() const;

private:
  std::size_t max_size_;
  std::deque<TimedTransform> samples_;
};

tf2::Transform computeMapToOdom(
  const tf2::Transform & map_to_base,
  const tf2::Transform & odom_to_base);

bool isFiniteTransform(const tf2::Transform & transform);

}  // namespace rm_relocalization_bridge
