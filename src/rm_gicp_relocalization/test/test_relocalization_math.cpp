#include <cmath>

#include <Eigen/Geometry>
#include "gtest/gtest.h"

#include "rm_gicp_relocalization/relocalization_math.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TEST(GicpRelocalizationMath, InitialPoseProducesMapOdomCorrection)
{
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  map_to_base.translation() << 3.0, -1.0, 0.0;
  map_to_base.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
  odom_to_base.translation() << 0.5, 0.2, 0.0;
  odom_to_base.linear() = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto correction = rm_gicp_relocalization::mapToOdomFromInitialPose(
    map_to_base, odom_to_base);
  const auto recovered = rm_gicp_relocalization::mapToBaseFromCorrection(
    correction, odom_to_base);

  EXPECT_TRUE(recovered.matrix().isApprox(map_to_base.matrix(), 1.0e-9));
}

TEST(GicpRelocalizationMath, YawDifferenceWrapsAtPi)
{
  Eigen::Isometry3d first = Eigen::Isometry3d::Identity();
  first.linear() = Eigen::AngleAxisd(3.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d second = Eigen::Isometry3d::Identity();
  second.linear() = Eigen::AngleAxisd(-3.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_NEAR(
    std::abs(rm_gicp_relocalization::planarYawDifference(first, second)),
    2.0 * kPi - 6.2,
    1.0e-9);
}
