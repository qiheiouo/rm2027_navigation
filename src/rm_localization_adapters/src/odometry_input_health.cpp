#include "rm_localization_adapters/odometry_input_health.hpp"

#include <cmath>
#include <stdexcept>

namespace rm_localization_adapters
{

OdometryInputHealthGate::OdometryInputHealthGate(
  const OdometryInputHealthConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.max_message_age_sec) ||
    config_.max_message_age_sec <= 0.0 ||
    !std::isfinite(config_.max_future_offset_sec) ||
    config_.max_future_offset_sec < 0.0 ||
    config_.recovery_fresh_samples == 0U ||
    !std::isfinite(config_.recovery_max_translation_m) ||
    config_.recovery_max_translation_m <= 0.0)
  {
    throw std::invalid_argument("invalid odometry input health configuration");
  }
}

OdometryInputDecision OdometryInputHealthGate::evaluate(
  std::int64_t receipt_time_ns,
  std::int64_t message_stamp_ns,
  double x,
  double y,
  double z)
{
  constexpr double kNanosecondsPerSecond = 1.0e9;
  last_age_sec_ = static_cast<double>(receipt_time_ns - message_stamp_ns) /
    kNanosecondsPerSecond;

  if (!config_.enabled) {
    set_anchor(x, y, z);
    return OdometryInputDecision::kAccept;
  }
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    message_stamp_ns <= 0 || last_age_sec_ > config_.max_message_age_sec)
  {
    latch();
    return OdometryInputDecision::kRejectStale;
  }
  if (last_age_sec_ < -config_.max_future_offset_sec) {
    latch();
    return OdometryInputDecision::kRejectFuture;
  }

  if (!has_anchor_) {
    set_anchor(x, y, z);
  }
  if (!latched_) {
    set_anchor(x, y, z);
    return OdometryInputDecision::kAccept;
  }

  const double dx = x - anchor_x_;
  const double dy = y - anchor_y_;
  const double dz = z - anchor_z_;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (distance > config_.recovery_max_translation_m) {
    recovery_count_ = 0U;
    return OdometryInputDecision::kRejectAnchor;
  }

  ++recovery_count_;
  if (recovery_count_ < config_.recovery_fresh_samples) {
    return OdometryInputDecision::kRecovering;
  }

  latched_ = false;
  recovery_count_ = 0U;
  set_anchor(x, y, z);
  return OdometryInputDecision::kAccept;
}

void OdometryInputHealthGate::latch_silence()
{
  if (config_.enabled) {
    latch();
  }
}

bool OdometryInputHealthGate::latched() const
{
  return latched_;
}

bool OdometryInputHealthGate::has_anchor() const
{
  return has_anchor_;
}

std::uint32_t OdometryInputHealthGate::recovery_count() const
{
  return recovery_count_;
}

double OdometryInputHealthGate::last_age_sec() const
{
  return last_age_sec_;
}

void OdometryInputHealthGate::latch()
{
  latched_ = true;
  recovery_count_ = 0U;
}

void OdometryInputHealthGate::set_anchor(double x, double y, double z)
{
  anchor_x_ = x;
  anchor_y_ = y;
  anchor_z_ = z;
  has_anchor_ = true;
}

}  // namespace rm_localization_adapters
