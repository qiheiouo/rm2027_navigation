#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include "gtest/gtest.h"

#include "rm_gicp_relocalization/registration_quality.hpp"
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

TEST(GicpRegistrationQuality, ThreeDimensionalStructurePasses)
{
  using rm_gicp_relocalization::RegistrationQualitySample;
  const std::vector<RegistrationQualitySample> samples{
    {{1.0, -1.0, -1.0}, {1.0, 0.0, 0.0}},
    {{1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}},
    {{-1.0, -1.0, 1.0}, {-1.0, 0.0, 0.0}},
    {{-1.0, 1.0, -1.0}, {-1.0, 0.0, 0.0}},
    {{-1.0, 1.0, -1.0}, {0.0, 1.0, 0.0}},
    {{1.0, 1.0, 1.0}, {0.0, 1.0, 0.0}},
    {{1.0, -1.0, -1.0}, {0.0, -1.0, 0.0}},
    {{-1.0, -1.0, 1.0}, {0.0, -1.0, 0.0}},
    {{-1.0, -1.0, 1.0}, {0.0, 0.0, 1.0}},
    {{1.0, 1.0, 1.0}, {0.0, 0.0, 1.0}},
    {{1.0, -1.0, -1.0}, {0.0, 0.0, -1.0}},
    {{-1.0, 1.0, -1.0}, {0.0, 0.0, -1.0}},
  };

  const auto metrics = rm_gicp_relocalization::calculateRegistrationQuality(samples, 12);
  EXPECT_TRUE(metrics.information_valid);
  EXPECT_DOUBLE_EQ(metrics.overlap_ratio, 1.0);
  EXPECT_GT(metrics.min_information_eigenvalue, 0.0);
  EXPECT_TRUE(std::isfinite(metrics.information_condition_number));

  rm_gicp_relocalization::RegistrationQualityThresholds thresholds;
  thresholds.min_overlap_ratio = 0.8;
  thresholds.min_information_eigenvalue = 1.0e-3;
  thresholds.max_information_condition_number = 1.0e3;
  std::string reason;
  EXPECT_TRUE(rm_gicp_relocalization::passesRegistrationQuality(metrics, thresholds, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(GicpRegistrationQuality, SinglePlaneIsRejectedAsDegenerate)
{
  using rm_gicp_relocalization::RegistrationQualitySample;
  const std::vector<RegistrationQualitySample> samples{
    {{-1.0, -1.0, 0.0}, {0.0, 0.0, 1.0}},
    {{-1.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
    {{1.0, -1.0, 0.0}, {0.0, 0.0, 1.0}},
    {{1.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
  };
  const auto metrics = rm_gicp_relocalization::calculateRegistrationQuality(samples, 4);
  ASSERT_TRUE(metrics.information_valid);
  EXPECT_LE(metrics.min_information_eigenvalue, 0.0);
  EXPECT_FALSE(std::isfinite(metrics.information_condition_number));

  rm_gicp_relocalization::RegistrationQualityThresholds thresholds;
  std::string reason;
  EXPECT_FALSE(rm_gicp_relocalization::passesRegistrationQuality(metrics, thresholds, reason));
  EXPECT_EQ(reason, "information matrix is not positive definite");
}

TEST(GicpRegistrationQuality, LowOverlapIsRejectedBeforeInformationGate)
{
  using rm_gicp_relocalization::RegistrationQualitySample;
  const std::vector<RegistrationQualitySample> samples{
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
    {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
  };
  const auto metrics = rm_gicp_relocalization::calculateRegistrationQuality(samples, 10);
  EXPECT_DOUBLE_EQ(metrics.overlap_ratio, 0.2);

  rm_gicp_relocalization::RegistrationQualityThresholds thresholds;
  thresholds.min_overlap_ratio = 0.3;
  std::string reason;
  EXPECT_FALSE(rm_gicp_relocalization::passesRegistrationQuality(metrics, thresholds, reason));
  EXPECT_EQ(reason, "overlap ratio below threshold");
}

TEST(GicpRegistrationQuality, InvalidNormalFailsClosed)
{
  using rm_gicp_relocalization::RegistrationQualitySample;
  const std::vector<RegistrationQualitySample> samples{
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  };
  const auto metrics = rm_gicp_relocalization::calculateRegistrationQuality(samples, 1);
  EXPECT_FALSE(metrics.information_valid);

  rm_gicp_relocalization::RegistrationQualityThresholds thresholds;
  std::string reason;
  EXPECT_FALSE(rm_gicp_relocalization::passesRegistrationQuality(metrics, thresholds, reason));
  EXPECT_EQ(reason, "information matrix is invalid");
}

TEST(GicpRegistrationQuality, CenteringMakesSpectrumIndependentOfMapOrigin)
{
  using rm_gicp_relocalization::RegistrationQualitySample;
  const std::vector<RegistrationQualitySample> local_samples{
    {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
    {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}},
    {{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
    {{0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}},
    {{0.0, 0.0, -1.0}, {1.0, 0.0, 0.0}},
  };
  auto shifted_samples = local_samples;
  const Eigen::Vector3d map_offset(1000.0, -250.0, 40.0);
  for (auto & sample : shifted_samples) {
    sample.aligned_point += map_offset;
  }

  const auto local = rm_gicp_relocalization::calculateRegistrationQuality(local_samples, 6);
  const auto shifted = rm_gicp_relocalization::calculateRegistrationQuality(shifted_samples, 6);
  ASSERT_TRUE(local.information_valid);
  ASSERT_TRUE(shifted.information_valid);
  EXPECT_TRUE(
    local.information_eigenvalues.isApprox(
      shifted.information_eigenvalues, 1.0e-12));
}

TEST(GicpRegistrationQuality, ConditionNumberThresholdIsEnforced)
{
  using rm_gicp_relocalization::RegistrationQualitySample;
  const std::vector<RegistrationQualitySample> samples{
    {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
    {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}},
    {{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
    {{0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}},
    {{0.0, 0.0, -1.0}, {1.0, 0.0, 0.0}},
  };
  const auto metrics = rm_gicp_relocalization::calculateRegistrationQuality(samples, 6);
  ASSERT_TRUE(metrics.information_valid);
  ASSERT_TRUE(std::isfinite(metrics.information_condition_number));

  rm_gicp_relocalization::RegistrationQualityThresholds thresholds;
  thresholds.max_information_condition_number =
    metrics.information_condition_number * 0.5;
  std::string reason;
  EXPECT_FALSE(rm_gicp_relocalization::passesRegistrationQuality(metrics, thresholds, reason));
  EXPECT_EQ(reason, "information matrix is too ill-conditioned");
}
