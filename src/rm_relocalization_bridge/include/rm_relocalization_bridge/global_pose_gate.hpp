#pragma once

#include <cstdint>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace rm_relocalization_bridge
{

struct GlobalPoseGateLimits
{
  std::string map_frame{"map"};
  double max_pose_age_sec{0.5};
  double max_future_sec{0.1};
  double max_xy_variance{1.0};
  double max_yaw_variance{0.5};
  double max_abs_z{0.25};
  double max_abs_roll_pitch{0.15};
};

bool validateGlobalPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & message,
  std::int64_t now_nanoseconds,
  const GlobalPoseGateLimits & limits,
  std::string * reason = nullptr);

}  // namespace rm_relocalization_bridge
