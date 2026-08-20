#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace rm_gicp_relocalization
{

struct RegistrationQualitySample
{
  Eigen::Vector3d aligned_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_normal{Eigen::Vector3d::Zero()};
};

struct RegistrationQualityMetrics
{
  std::size_t source_points{0};
  std::size_t inliers{0};
  double overlap_ratio{0.0};
  Eigen::Matrix<double, 6, 1> information_eigenvalues{
    Eigen::Matrix<double, 6, 1>::Zero()};
  double min_information_eigenvalue{0.0};
  double max_information_eigenvalue{0.0};
  double information_condition_number{std::numeric_limits<double>::infinity()};
  bool information_valid{false};
};

struct RegistrationQualityThresholds
{
  double min_overlap_ratio{0.0};
  double min_information_eigenvalue{0.0};
  double max_information_condition_number{0.0};
};

RegistrationQualityMetrics calculateRegistrationQuality(
  const std::vector<RegistrationQualitySample> & samples,
  std::size_t source_points);

bool passesRegistrationQuality(
  const RegistrationQualityMetrics & metrics,
  const RegistrationQualityThresholds & thresholds,
  std::string & reason);

}  // namespace rm_gicp_relocalization
