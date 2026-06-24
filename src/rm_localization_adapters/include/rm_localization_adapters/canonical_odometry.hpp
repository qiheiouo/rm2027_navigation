#ifndef RM_LOCALIZATION_ADAPTERS__CANONICAL_ODOMETRY_HPP_
#define RM_LOCALIZATION_ADAPTERS__CANONICAL_ODOMETRY_HPP_

#include <cstdint>

#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Transform.h"

namespace rm_localization_adapters
{

tf2::Transform compute_base_transform(
  const tf2::Transform & odom_to_sensor,
  const tf2::Transform & base_to_sensor);

enum class TwistEstimateStatus
{
  kInitialized,
  kValid,
  kInvalidTime,
  kOutlier,
};

struct TwistEstimatorConfig
{
  double min_dt_sec{0.001};
  double max_dt_sec{0.5};
  double smoothing_alpha{0.3};
  double max_linear_speed{5.0};
  double max_angular_speed{20.0};
};

struct TwistEstimate
{
  TwistEstimateStatus status{TwistEstimateStatus::kInitialized};
  geometry_msgs::msg::Twist twist;
  double dt_sec{0.0};
};

class PoseTwistEstimator
{
public:
  explicit PoseTwistEstimator(const TwistEstimatorConfig & config);

  TwistEstimate update(
    const tf2::Transform & odom_to_base,
    std::int64_t stamp_nanoseconds);

  void reset();

private:
  void set_baseline(
    const tf2::Transform & odom_to_base,
    std::int64_t stamp_nanoseconds);

  TwistEstimatorConfig config_;
  bool initialized_{false};
  bool has_filtered_twist_{false};
  std::int64_t previous_stamp_nanoseconds_{0};
  tf2::Transform previous_odom_to_base_;
  geometry_msgs::msg::Twist filtered_twist_;
};

}  // namespace rm_localization_adapters

#endif  // RM_LOCALIZATION_ADAPTERS__CANONICAL_ODOMETRY_HPP_
