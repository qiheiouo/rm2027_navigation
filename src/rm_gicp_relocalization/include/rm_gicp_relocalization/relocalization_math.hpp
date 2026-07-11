#pragma once

#include <Eigen/Geometry>

namespace rm_gicp_relocalization
{

Eigen::Isometry3d mapToOdomFromInitialPose(
  const Eigen::Isometry3d & map_to_base,
  const Eigen::Isometry3d & odom_to_base);

Eigen::Isometry3d mapToBaseFromCorrection(
  const Eigen::Isometry3d & map_to_odom,
  const Eigen::Isometry3d & odom_to_base);

double planarYaw(const Eigen::Isometry3d & transform);
double planarYawDifference(
  const Eigen::Isometry3d & first,
  const Eigen::Isometry3d & second);

}  // namespace rm_gicp_relocalization
