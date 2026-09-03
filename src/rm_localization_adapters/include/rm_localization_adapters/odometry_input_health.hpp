#pragma once

#include <cstdint>

namespace rm_localization_adapters
{

enum class OdometryInputDecision
{
  kAccept,
  kRejectStale,
  kRejectFuture,
  kRecovering,
  kRejectAnchor,
};

struct OdometryInputHealthConfig
{
  bool enabled{false};
  double max_message_age_sec{0.2};
  double max_future_offset_sec{0.05};
  std::uint32_t recovery_fresh_samples{5U};
  double recovery_max_translation_m{0.75};
};

class OdometryInputHealthGate
{
public:
  explicit OdometryInputHealthGate(const OdometryInputHealthConfig & config);

  OdometryInputDecision evaluate(
    std::int64_t receipt_time_ns,
    std::int64_t message_stamp_ns,
    double x,
    double y,
    double z);

  void latch_silence();
  bool latched() const;
  bool has_anchor() const;
  std::uint32_t recovery_count() const;
  double last_age_sec() const;

private:
  void latch();
  void set_anchor(double x, double y, double z);

  OdometryInputHealthConfig config_;
  bool latched_{false};
  bool has_anchor_{false};
  std::uint32_t recovery_count_{0U};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_z_{0.0};
  double last_age_sec_{0.0};
};

}  // namespace rm_localization_adapters
