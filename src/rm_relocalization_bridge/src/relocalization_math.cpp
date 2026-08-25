#include "rm_relocalization_bridge/relocalization_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "tf2/LinearMath/Matrix3x3.h"

namespace rm_relocalization_bridge
{

TimedTransformCache::TimedTransformCache(std::size_t max_size)
: max_size_(max_size)
{
  if (max_size_ == 0U) {
    throw std::invalid_argument("TimedTransformCache max_size must be positive");
  }
}

bool TimedTransformCache::add(const TimedTransform & sample)
{
  bool monotonic = true;
  if (!samples_.empty() && sample.stamp_nanoseconds <= samples_.back().stamp_nanoseconds) {
    samples_.clear();
    monotonic = false;
  }

  samples_.push_back(sample);
  while (samples_.size() > max_size_) {
    samples_.pop_front();
  }
  return monotonic;
}

std::optional<TimedTransform> TimedTransformCache::nearest(
  std::int64_t stamp_nanoseconds,
  std::int64_t tolerance_nanoseconds) const
{
  if (samples_.empty() || tolerance_nanoseconds < 0) {
    return std::nullopt;
  }

  const TimedTransform * best = nullptr;
  std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();
  for (const auto & sample : samples_) {
    const auto delta = std::llabs(sample.stamp_nanoseconds - stamp_nanoseconds);
    if (delta < best_delta) {
      best_delta = delta;
      best = &sample;
    }
  }

  if (best == nullptr || best_delta > tolerance_nanoseconds) {
    return std::nullopt;
  }
  return *best;
}

void TimedTransformCache::clear()
{
  samples_.clear();
}

std::size_t TimedTransformCache::size() const
{
  return samples_.size();
}

tf2::Transform computeMapToOdom(
  const tf2::Transform & map_to_base,
  const tf2::Transform & odom_to_base)
{
  return map_to_base * odom_to_base.inverse();
}

bool isFiniteTransform(const tf2::Transform & transform)
{
  const auto & origin = transform.getOrigin();
  const auto & rotation = transform.getRotation();
  return
    std::isfinite(origin.x()) &&
    std::isfinite(origin.y()) &&
    std::isfinite(origin.z()) &&
    std::isfinite(rotation.x()) &&
    std::isfinite(rotation.y()) &&
    std::isfinite(rotation.z()) &&
    std::isfinite(rotation.w()) &&
    rotation.length2() > 1.0e-12;
}

CorrectionInnovation measureCorrectionInnovation(
  const tf2::Transform & previous_map_to_odom,
  const tf2::Transform & candidate_map_to_odom)
{
  const auto translation_delta =
    candidate_map_to_odom.getOrigin() - previous_map_to_odom.getOrigin();

  double previous_roll = 0.0;
  double previous_pitch = 0.0;
  double previous_yaw = 0.0;
  tf2::Matrix3x3(previous_map_to_odom.getRotation()).getRPY(
    previous_roll, previous_pitch, previous_yaw);

  double candidate_roll = 0.0;
  double candidate_pitch = 0.0;
  double candidate_yaw = 0.0;
  tf2::Matrix3x3(candidate_map_to_odom.getRotation()).getRPY(
    candidate_roll, candidate_pitch, candidate_yaw);

  (void)previous_roll;
  (void)previous_pitch;
  (void)candidate_roll;
  (void)candidate_pitch;
  const double yaw_delta = std::atan2(
    std::sin(candidate_yaw - previous_yaw),
    std::cos(candidate_yaw - previous_yaw));

  return {
    std::hypot(translation_delta.x(), translation_delta.y()),
    std::abs(yaw_delta),
  };
}

}  // namespace rm_relocalization_bridge
