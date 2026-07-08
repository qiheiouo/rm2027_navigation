#include "rm_localization_adapters/canonical_odometry.hpp"

#include <cmath>
#include <stdexcept>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace rm_localization_adapters
{

namespace
{

geometry_msgs::msg::Twist blend_twist(
  const geometry_msgs::msg::Twist & previous,
  const geometry_msgs::msg::Twist & current,
  double alpha)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = alpha * current.linear.x + (1.0 - alpha) * previous.linear.x;
  result.linear.y = alpha * current.linear.y + (1.0 - alpha) * previous.linear.y;
  result.linear.z = alpha * current.linear.z + (1.0 - alpha) * previous.linear.z;
  result.angular.x = alpha * current.angular.x + (1.0 - alpha) * previous.angular.x;
  result.angular.y = alpha * current.angular.y + (1.0 - alpha) * previous.angular.y;
  result.angular.z = alpha * current.angular.z + (1.0 - alpha) * previous.angular.z;
  return result;
}

geometry_msgs::msg::Twist rotate_twist(
  const geometry_msgs::msg::Twist & input,
  const tf2::Quaternion & target_from_source)
{
  const tf2::Vector3 linear = tf2::quatRotate(
    target_from_source,
    tf2::Vector3(input.linear.x, input.linear.y, input.linear.z));
  const tf2::Vector3 angular = tf2::quatRotate(
    target_from_source,
    tf2::Vector3(input.angular.x, input.angular.y, input.angular.z));

  geometry_msgs::msg::Twist result;
  result.linear.x = linear.x();
  result.linear.y = linear.y();
  result.linear.z = linear.z();
  result.angular.x = angular.x();
  result.angular.y = angular.y();
  result.angular.z = angular.z();
  return result;
}

}  // namespace

tf2::Transform compute_base_transform(
  const tf2::Transform & odom_to_sensor,
  const tf2::Transform & base_to_sensor)
{
  return odom_to_sensor * base_to_sensor.inverse();
}

tf2::Transform compute_base_transform_from_sensor_initial(
  const tf2::Transform & sensor_initial_to_sensor,
  const tf2::Transform & base_to_sensor)
{
  return base_to_sensor * sensor_initial_to_sensor * base_to_sensor.inverse();
}

PoseTwistEstimator::PoseTwistEstimator(const TwistEstimatorConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.min_dt_sec) || !std::isfinite(config_.max_dt_sec) ||
    config_.min_dt_sec <= 0.0 || config_.max_dt_sec <= config_.min_dt_sec)
  {
    throw std::invalid_argument("twist estimator requires 0 < min_dt < max_dt");
  }
  if (!std::isfinite(config_.smoothing_alpha) ||
    config_.smoothing_alpha <= 0.0 || config_.smoothing_alpha > 1.0)
  {
    throw std::invalid_argument("twist smoothing_alpha must be in (0, 1]");
  }
  if (!std::isfinite(config_.max_linear_speed) ||
    !std::isfinite(config_.max_angular_speed) ||
    config_.max_linear_speed <= 0.0 || config_.max_angular_speed <= 0.0)
  {
    throw std::invalid_argument("twist outlier limits must be positive");
  }
}

TwistEstimate PoseTwistEstimator::update(
  const tf2::Transform & odom_to_base,
  std::int64_t stamp_nanoseconds)
{
  TwistEstimate result;
  if (!initialized_) {
    set_baseline(odom_to_base, stamp_nanoseconds);
    return result;
  }

  result.dt_sec = static_cast<double>(
    stamp_nanoseconds - previous_stamp_nanoseconds_) * 1.0e-9;
  if (result.dt_sec < config_.min_dt_sec || result.dt_sec > config_.max_dt_sec) {
    result.status = TwistEstimateStatus::kInvalidTime;
    set_baseline(odom_to_base, stamp_nanoseconds);
    has_filtered_twist_ = false;
    return result;
  }

  const tf2::Vector3 world_linear_velocity =
    (odom_to_base.getOrigin() - previous_odom_to_base_.getOrigin()) / result.dt_sec;
  const tf2::Vector3 base_linear_velocity = tf2::quatRotate(
    odom_to_base.getRotation().inverse(), world_linear_velocity);

  tf2::Quaternion delta_rotation =
    previous_odom_to_base_.getRotation().inverse() * odom_to_base.getRotation();
  delta_rotation.normalize();
  if (delta_rotation.w() < 0.0) {
    delta_rotation = tf2::Quaternion(
      -delta_rotation.x(), -delta_rotation.y(), -delta_rotation.z(), -delta_rotation.w());
  }

  const double rotation_angle = delta_rotation.getAngle();
  tf2::Vector3 base_angular_velocity(0.0, 0.0, 0.0);
  if (rotation_angle > 1.0e-12) {
    base_angular_velocity = delta_rotation.getAxis() * (rotation_angle / result.dt_sec);
  }

  geometry_msgs::msg::Twist raw_twist;
  raw_twist.linear.x = base_linear_velocity.x();
  raw_twist.linear.y = base_linear_velocity.y();
  raw_twist.linear.z = base_linear_velocity.z();
  raw_twist.angular.x = base_angular_velocity.x();
  raw_twist.angular.y = base_angular_velocity.y();
  raw_twist.angular.z = base_angular_velocity.z();

  const bool has_finite_velocity =
    std::isfinite(base_linear_velocity.x()) &&
    std::isfinite(base_linear_velocity.y()) &&
    std::isfinite(base_linear_velocity.z()) &&
    std::isfinite(base_angular_velocity.x()) &&
    std::isfinite(base_angular_velocity.y()) &&
    std::isfinite(base_angular_velocity.z());
  if (!has_finite_velocity) {
    result.status = TwistEstimateStatus::kOutlier;
    has_filtered_twist_ = false;
    return result;
  }

  if (base_linear_velocity.length() > config_.max_linear_speed ||
    base_angular_velocity.length() > config_.max_angular_speed)
  {
    result.status = TwistEstimateStatus::kOutlier;
    set_baseline(odom_to_base, stamp_nanoseconds);
    has_filtered_twist_ = false;
    return result;
  }

  if (has_filtered_twist_) {
    const tf2::Quaternion current_from_previous =
      odom_to_base.getRotation().inverse() * previous_odom_to_base_.getRotation();
    const auto previous_twist_in_current = rotate_twist(
      filtered_twist_, current_from_previous);
    filtered_twist_ = blend_twist(
      previous_twist_in_current, raw_twist, config_.smoothing_alpha);
  } else {
    filtered_twist_ = raw_twist;
    has_filtered_twist_ = true;
  }

  result.status = TwistEstimateStatus::kValid;
  result.twist = filtered_twist_;
  set_baseline(odom_to_base, stamp_nanoseconds);
  return result;
}

void PoseTwistEstimator::reset()
{
  initialized_ = false;
  has_filtered_twist_ = false;
  previous_stamp_nanoseconds_ = 0;
  previous_odom_to_base_.setIdentity();
  filtered_twist_ = geometry_msgs::msg::Twist();
}

void PoseTwistEstimator::set_baseline(
  const tf2::Transform & odom_to_base,
  std::int64_t stamp_nanoseconds)
{
  previous_odom_to_base_ = odom_to_base;
  previous_stamp_nanoseconds_ = stamp_nanoseconds;
  initialized_ = true;
}

}  // namespace rm_localization_adapters
