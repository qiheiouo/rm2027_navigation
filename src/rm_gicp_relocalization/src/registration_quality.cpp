#include "rm_gicp_relocalization/registration_quality.hpp"

#include <cmath>

#include <Eigen/Eigenvalues>

namespace rm_gicp_relocalization
{

RegistrationQualityMetrics calculateRegistrationQuality(
  const std::vector<RegistrationQualitySample> & samples,
  const std::size_t source_points)
{
  RegistrationQualityMetrics metrics;
  metrics.source_points = source_points;
  metrics.inliers = samples.size();
  if (source_points == 0 || samples.empty() || samples.size() > source_points) {
    return metrics;
  }
  metrics.overlap_ratio =
    static_cast<double>(samples.size()) / static_cast<double>(source_points);

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & sample : samples) {
    const double normal_norm = sample.target_normal.norm();
    if (
      !sample.aligned_point.allFinite() || !sample.target_normal.allFinite() ||
      !std::isfinite(normal_norm) || normal_norm <= 1.0e-12)
    {
      return metrics;
    }
    centroid += sample.aligned_point;
  }
  centroid /= static_cast<double>(samples.size());

  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  for (const auto & sample : samples) {
    const Eigen::Vector3d normal = sample.target_normal.normalized();
    const Eigen::Vector3d lever_arm = sample.aligned_point - centroid;
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian << normal.transpose(), lever_arm.cross(normal).transpose();
    information.noalias() += jacobian.transpose() * jacobian;
  }
  information /= static_cast<double>(samples.size());
  information = 0.5 * (information + information.transpose());
  if (!information.allFinite()) {
    return metrics;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return metrics;
  }
  metrics.information_valid = true;
  metrics.information_eigenvalues = solver.eigenvalues();
  metrics.min_information_eigenvalue = metrics.information_eigenvalues.minCoeff();
  metrics.max_information_eigenvalue = metrics.information_eigenvalues.maxCoeff();
  if (metrics.min_information_eigenvalue > 0.0) {
    metrics.information_condition_number =
      metrics.max_information_eigenvalue / metrics.min_information_eigenvalue;
  }
  return metrics;
}

bool passesRegistrationQuality(
  const RegistrationQualityMetrics & metrics,
  const RegistrationQualityThresholds & thresholds,
  std::string & reason)
{
  if (
    !std::isfinite(metrics.overlap_ratio) ||
    metrics.overlap_ratio < thresholds.min_overlap_ratio)
  {
    reason = "overlap ratio below threshold";
    return false;
  }
  if (!metrics.information_valid) {
    reason = "information matrix is invalid";
    return false;
  }
  if (
    !std::isfinite(metrics.min_information_eigenvalue) ||
    metrics.min_information_eigenvalue <= 0.0)
  {
    reason = "information matrix is not positive definite";
    return false;
  }
  if (metrics.min_information_eigenvalue < thresholds.min_information_eigenvalue) {
    reason = "minimum information eigenvalue below threshold";
    return false;
  }
  if (!std::isfinite(metrics.information_condition_number)) {
    reason = "information matrix condition number is invalid";
    return false;
  }
  if (
    thresholds.max_information_condition_number > 0.0 &&
    metrics.information_condition_number > thresholds.max_information_condition_number)
  {
    reason = "information matrix is too ill-conditioned";
    return false;
  }
  reason.clear();
  return true;
}

}  // namespace rm_gicp_relocalization
