#include "rm_gicp_relocalization/relocalization_math.hpp"

#include <cmath>

namespace rm_gicp_relocalization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

Eigen::Isometry3d mapToOdomFromInitialPose(
  const Eigen::Isometry3d & map_to_base,
  const Eigen::Isometry3d & odom_to_base)
{
  return map_to_base * odom_to_base.inverse();
}

Eigen::Isometry3d mapToBaseFromCorrection(
  const Eigen::Isometry3d & map_to_odom,
  const Eigen::Isometry3d & odom_to_base)
{
  return map_to_odom * odom_to_base;
}

double planarYaw(const Eigen::Isometry3d & transform)
{
  return std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));
}

double planarYawDifference(
  const Eigen::Isometry3d & first,
  const Eigen::Isometry3d & second)
{
  return std::remainder(planarYaw(first) - planarYaw(second), 2.0 * kPi);
}

}  // namespace rm_gicp_relocalization
