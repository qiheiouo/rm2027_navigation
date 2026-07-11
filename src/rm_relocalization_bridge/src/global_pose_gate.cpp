#include "rm_relocalization_bridge/global_pose_gate.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace rm_relocalization_bridge
{
namespace
{

bool reject(std::string * reason, const std::string & text)
{
  if (reason != nullptr) {
    *reason = text;
  }
  return false;
}

}  // namespace

bool validateGlobalPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & message,
  const std::int64_t now_nanoseconds,
  const GlobalPoseGateLimits & limits,
  std::string * reason)
{
  if (message.header.frame_id != limits.map_frame) {
    return reject(reason, "wrong frame");
  }

  const std::int64_t stamp_nanoseconds =
    static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(message.header.stamp.nanosec);
  if (stamp_nanoseconds <= 0) {
    return reject(reason, "zero timestamp");
  }
  if (now_nanoseconds > 0) {
    const double age = static_cast<double>(now_nanoseconds - stamp_nanoseconds) * 1.0e-9;
    if (age > limits.max_pose_age_sec) {
      return reject(reason, "stale timestamp");
    }
    if (age < -limits.max_future_sec) {
      return reject(reason, "future timestamp");
    }
  }

  const auto & pose = message.pose.pose;
  const std::array<double, 7> pose_values{
    pose.position.x,
    pose.position.y,
    pose.position.z,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w,
  };
  if (!std::all_of(pose_values.begin(), pose_values.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    return reject(reason, "non-finite pose");
  }
  if (std::abs(pose.position.z) > limits.max_abs_z) {
    return reject(reason, "non-planar height");
  }

  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return reject(reason, "invalid quaternion");
  }
  const double x = pose.orientation.x / norm;
  const double y = pose.orientation.y / norm;
  const double z = pose.orientation.z / norm;
  const double w = pose.orientation.w / norm;
  const double roll = std::atan2(
    2.0 * (w * x + y * z),
    1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  if (
    std::abs(roll) > limits.max_abs_roll_pitch ||
    std::abs(pitch) > limits.max_abs_roll_pitch)
  {
    return reject(reason, "non-planar orientation");
  }

  const auto & covariance = message.pose.covariance;
  if (!std::all_of(covariance.begin(), covariance.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    return reject(reason, "non-finite covariance");
  }
  const double variance_x = covariance[0];
  const double variance_y = covariance[7];
  const double variance_yaw = covariance[35];
  if (variance_x < 0.0 || variance_y < 0.0 || variance_yaw < 0.0) {
    return reject(reason, "negative covariance");
  }
  if (variance_x > limits.max_xy_variance || variance_y > limits.max_xy_variance) {
    return reject(reason, "xy covariance exceeds limit");
  }
  if (variance_yaw > limits.max_yaw_variance) {
    return reject(reason, "yaw covariance exceeds limit");
  }

  if (reason != nullptr) {
    reason->clear();
  }
  return true;
}

}  // namespace rm_relocalization_bridge
