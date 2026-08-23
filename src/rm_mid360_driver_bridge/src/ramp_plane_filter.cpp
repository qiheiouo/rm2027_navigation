#include "rm_mid360_driver_bridge/ramp_plane_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm_mid360_driver_bridge
{
namespace
{

constexpr double kGeometryEpsilon = 1e-9;

bool finite_point(const Point2d & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool point_on_segment(
  const Point2d & first,
  const Point2d & second,
  double x,
  double y)
{
  const double segment_x = second.x - first.x;
  const double segment_y = second.y - first.y;
  const double point_x = x - first.x;
  const double point_y = y - first.y;
  const double cross = segment_x * point_y - segment_y * point_x;
  if (std::abs(cross) > kGeometryEpsilon) {
    return false;
  }
  const double dot = point_x * segment_x + point_y * segment_y;
  if (dot < -kGeometryEpsilon) {
    return false;
  }
  const double squared_length = segment_x * segment_x + segment_y * segment_y;
  return dot <= squared_length + kGeometryEpsilon;
}

}  // namespace

bool validate_ramp_region(const RampRegion & region, std::string & error)
{
  if (region.name.empty()) {
    error = "ramp region name must not be empty";
    return false;
  }
  if (region.polygon.size() < 3U) {
    error = "ramp region " + region.name + " requires at least three polygon vertices";
    return false;
  }
  if (!std::all_of(region.polygon.begin(), region.polygon.end(), finite_point)) {
    error = "ramp region " + region.name + " polygon contains a non-finite vertex";
    return false;
  }
  if (!std::isfinite(region.origin_x) || !std::isfinite(region.origin_y) ||
    !std::isfinite(region.origin_height) || !std::isfinite(region.ascent_yaw_rad) ||
    !std::isfinite(region.slope_rad) || !std::isfinite(region.surface_tolerance))
  {
    error = "ramp region " + region.name + " contains a non-finite plane parameter";
    return false;
  }
  if (std::abs(region.slope_rad) >= 0.7853981633974483) {
    error = "ramp region " + region.name + " slope must stay below 45 degrees";
    return false;
  }
  if (region.surface_tolerance <= 0.0 || region.surface_tolerance > 0.20) {
    error = "ramp region " + region.name + " surface tolerance must be in (0, 0.20] m";
    return false;
  }

  double signed_twice_area = 0.0;
  for (std::size_t index = 0; index < region.polygon.size(); ++index) {
    const auto & current = region.polygon[index];
    const auto & next = region.polygon[(index + 1U) % region.polygon.size()];
    signed_twice_area += current.x * next.y - next.x * current.y;
  }
  if (std::abs(signed_twice_area) <= kGeometryEpsilon) {
    error = "ramp region " + region.name + " polygon area must be non-zero";
    return false;
  }

  error.clear();
  return true;
}

bool point_in_polygon(
  const std::vector<Point2d> & polygon,
  double x,
  double y)
{
  if (polygon.size() < 3U || !std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }

  bool inside = false;
  for (std::size_t current = 0, previous = polygon.size() - 1U;
    current < polygon.size(); previous = current++)
  {
    const auto & first = polygon[previous];
    const auto & second = polygon[current];
    if (point_on_segment(first, second, x, y)) {
      return true;
    }

    const bool crosses_y = (first.y > y) != (second.y > y);
    if (!crosses_y) {
      continue;
    }
    const double crossing_x =
      (second.x - first.x) * (y - first.y) / (second.y - first.y) + first.x;
    if (x < crossing_x) {
      inside = !inside;
    }
  }
  return inside;
}

double expected_ramp_height(
  const RampRegion & region,
  double x,
  double y)
{
  const double delta_x = x - region.origin_x;
  const double delta_y = y - region.origin_y;
  const double distance_along_ascent =
    std::cos(region.ascent_yaw_rad) * delta_x +
    std::sin(region.ascent_yaw_rad) * delta_y;
  return region.origin_height + std::tan(region.slope_rad) * distance_along_ascent;
}

std::optional<RampSurfaceMatch> match_expected_ramp_surface(
  const std::vector<RampRegion> & regions,
  double x,
  double y,
  double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return std::nullopt;
  }

  for (std::size_t index = 0; index < regions.size(); ++index) {
    const auto & region = regions[index];
    if (!point_in_polygon(region.polygon, x, y)) {
      continue;
    }
    const double expected_height = expected_ramp_height(region, x, y);
    const double residual = z - expected_height;
    if (std::abs(residual) <= region.surface_tolerance) {
      return RampSurfaceMatch{index, expected_height, residual};
    }
  }
  return std::nullopt;
}

}  // namespace rm_mid360_driver_bridge
