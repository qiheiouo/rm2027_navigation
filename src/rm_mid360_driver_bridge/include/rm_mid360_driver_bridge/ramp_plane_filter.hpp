#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace rm_mid360_driver_bridge
{

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

struct Point3d
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct RampRegion
{
  std::string name;
  std::vector<Point2d> polygon;
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_height{0.0};
  double ascent_yaw_rad{0.0};
  double slope_rad{0.0};
  double surface_tolerance{0.04};
};

struct RampSurfaceMatch
{
  std::size_t region_index{0};
  double expected_height{0.0};
  double height_residual{0.0};
};

struct AutomaticRampDetectionConfig
{
  double min_slope_rad{0.08726646259971647};
  double max_slope_rad{0.4363323129985824};
  double inlier_tolerance{0.025};
  double surface_tolerance{0.05};
  std::size_t min_inliers{30U};
  double min_inlier_ratio{0.12};
  double min_length{0.60};
  double min_width{0.55};
  double min_height_span{0.08};
  std::size_t ransac_iterations{256U};
};

struct AutomaticRampDetection
{
  RampRegion region;
  std::size_t inlier_count{0U};
  double inlier_ratio{0.0};
  double rms_residual{0.0};
  double length{0.0};
  double width{0.0};
  double height_span{0.0};
};

bool validate_ramp_region(const RampRegion & region, std::string & error);

bool point_in_polygon(
  const std::vector<Point2d> & polygon,
  double x,
  double y);

double expected_ramp_height(
  const RampRegion & region,
  double x,
  double y);

std::optional<RampSurfaceMatch> match_expected_ramp_surface(
  const std::vector<RampRegion> & regions,
  double x,
  double y,
  double z);

std::optional<AutomaticRampDetection> detect_automatic_ramp(
  const std::vector<Point3d> & points,
  const AutomaticRampDetectionConfig & config);

}  // namespace rm_mid360_driver_bridge
