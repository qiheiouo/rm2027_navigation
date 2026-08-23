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

}  // namespace rm_mid360_driver_bridge
