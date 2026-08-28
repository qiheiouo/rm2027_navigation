#include "rm_mid360_driver_bridge/ramp_plane_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace rm_mid360_driver_bridge
{
namespace
{

constexpr double kGeometryEpsilon = 1e-9;

bool finite_point(const Point2d & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finite_point(const Point3d & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

struct HeightPlane
{
  double a{0.0};
  double b{0.0};
  double c{0.0};
};

bool plane_from_three_points(
  const Point3d & first,
  const Point3d & second,
  const Point3d & third,
  HeightPlane & plane)
{
  const double ux = second.x - first.x;
  const double uy = second.y - first.y;
  const double uz = second.z - first.z;
  const double vx = third.x - first.x;
  const double vy = third.y - first.y;
  const double vz = third.z - first.z;
  const double nx = uy * vz - uz * vy;
  const double ny = uz * vx - ux * vz;
  const double nz = ux * vy - uy * vx;
  if (std::abs(nz) <= kGeometryEpsilon) {
    return false;
  }
  plane.a = -nx / nz;
  plane.b = -ny / nz;
  plane.c = first.z - plane.a * first.x - plane.b * first.y;
  return std::isfinite(plane.a) && std::isfinite(plane.b) && std::isfinite(plane.c);
}

double vertical_residual(const HeightPlane & plane, const Point3d & point)
{
  return point.z - (plane.a * point.x + plane.b * point.y + plane.c);
}

bool solve_three_by_three(double matrix[3][4], HeightPlane & plane)
{
  for (std::size_t pivot = 0U; pivot < 3U; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1U; row < 3U; ++row) {
      if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot])) {
        best = row;
      }
    }
    if (std::abs(matrix[best][pivot]) <= kGeometryEpsilon) {
      return false;
    }
    if (best != pivot) {
      for (std::size_t column = pivot; column < 4U; ++column) {
        std::swap(matrix[pivot][column], matrix[best][column]);
      }
    }
    const double divisor = matrix[pivot][pivot];
    for (std::size_t column = pivot; column < 4U; ++column) {
      matrix[pivot][column] /= divisor;
    }
    for (std::size_t row = 0U; row < 3U; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = matrix[row][pivot];
      for (std::size_t column = pivot; column < 4U; ++column) {
        matrix[row][column] -= factor * matrix[pivot][column];
      }
    }
  }
  plane.a = matrix[0][3];
  plane.b = matrix[1][3];
  plane.c = matrix[2][3];
  return std::isfinite(plane.a) && std::isfinite(plane.b) && std::isfinite(plane.c);
}

bool least_squares_plane(
  const std::vector<Point3d> & points,
  const std::vector<std::size_t> & indices,
  HeightPlane & plane)
{
  if (indices.size() < 3U) {
    return false;
  }
  double xx = 0.0;
  double xy = 0.0;
  double x = 0.0;
  double yy = 0.0;
  double y = 0.0;
  double xz = 0.0;
  double yz = 0.0;
  double z = 0.0;
  for (const auto index : indices) {
    const auto & point = points[index];
    xx += point.x * point.x;
    xy += point.x * point.y;
    x += point.x;
    yy += point.y * point.y;
    y += point.y;
    xz += point.x * point.z;
    yz += point.y * point.z;
    z += point.z;
  }
  double system[3][4] = {
    {xx, xy, x, xz},
    {xy, yy, y, yz},
    {x, y, static_cast<double>(indices.size()), z},
  };
  return solve_three_by_three(system, plane);
}

double quantile(std::vector<double> values, double fraction)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
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
  if (!std::all_of(
      region.polygon.begin(), region.polygon.end(),
      [](const Point2d & point) {return finite_point(point);}))
  {
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

std::optional<AutomaticRampDetection> detect_automatic_ramp(
  const std::vector<Point3d> & points,
  const AutomaticRampDetectionConfig & config)
{
  if (points.size() < std::max<std::size_t>(config.min_inliers, 3U) ||
    config.min_slope_rad <= 0.0 || config.max_slope_rad <= config.min_slope_rad ||
    config.max_slope_rad >= 0.7853981633974483 || config.inlier_tolerance <= 0.0 ||
    config.surface_tolerance <= 0.0 || config.surface_tolerance > 0.20 ||
    config.min_inlier_ratio <= 0.0 || config.min_inlier_ratio > 1.0 ||
    config.min_length <= 0.0 || config.min_width <= 0.0 ||
    config.min_height_span <= 0.0 || config.ransac_iterations == 0U ||
    !std::all_of(
      points.begin(), points.end(),
      [](const Point3d & point) {return finite_point(point);}))
  {
    return std::nullopt;
  }

  HeightPlane best_plane;
  std::vector<std::size_t> best_inliers;
  double best_squared_error = std::numeric_limits<double>::infinity();
  std::minstd_rand generator(static_cast<unsigned int>(points.size() * 2654435761U));
  std::uniform_int_distribution<std::size_t> distribution(0U, points.size() - 1U);
  for (std::size_t iteration = 0U; iteration < config.ransac_iterations; ++iteration) {
    const std::size_t first = distribution(generator);
    const std::size_t second = distribution(generator);
    const std::size_t third = distribution(generator);
    if (first == second || first == third || second == third) {
      continue;
    }
    HeightPlane candidate;
    if (!plane_from_three_points(points[first], points[second], points[third], candidate)) {
      continue;
    }
    const double slope = std::atan(std::hypot(candidate.a, candidate.b));
    if (slope < config.min_slope_rad || slope > config.max_slope_rad) {
      continue;
    }
    std::vector<std::size_t> inliers;
    inliers.reserve(points.size());
    double squared_error = 0.0;
    for (std::size_t index = 0U; index < points.size(); ++index) {
      const double residual = vertical_residual(candidate, points[index]);
      if (std::abs(residual) <= config.inlier_tolerance) {
        inliers.push_back(index);
        squared_error += residual * residual;
      }
    }
    if (inliers.size() > best_inliers.size() ||
      (inliers.size() == best_inliers.size() && squared_error < best_squared_error))
    {
      best_plane = candidate;
      best_inliers = std::move(inliers);
      best_squared_error = squared_error;
    }
  }

  if (best_inliers.size() < config.min_inliers ||
    static_cast<double>(best_inliers.size()) / static_cast<double>(points.size()) <
    config.min_inlier_ratio)
  {
    return std::nullopt;
  }
  if (!least_squares_plane(points, best_inliers, best_plane)) {
    return std::nullopt;
  }
  const double gradient = std::hypot(best_plane.a, best_plane.b);
  const double slope = std::atan(gradient);
  if (gradient <= kGeometryEpsilon || slope < config.min_slope_rad ||
    slope > config.max_slope_rad)
  {
    return std::nullopt;
  }

  best_inliers.clear();
  double squared_error = 0.0;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const double residual = vertical_residual(best_plane, points[index]);
    if (std::abs(residual) <= config.inlier_tolerance) {
      best_inliers.push_back(index);
      squared_error += residual * residual;
    }
  }
  if (best_inliers.size() < config.min_inliers ||
    static_cast<double>(best_inliers.size()) / static_cast<double>(points.size()) <
    config.min_inlier_ratio)
  {
    return std::nullopt;
  }

  const double ascent_x = best_plane.a / gradient;
  const double ascent_y = best_plane.b / gradient;
  const double cross_x = -ascent_y;
  const double cross_y = ascent_x;
  std::vector<double> along_values;
  std::vector<double> cross_values;
  along_values.reserve(best_inliers.size());
  cross_values.reserve(best_inliers.size());
  for (const auto index : best_inliers) {
    const auto & point = points[index];
    along_values.push_back(ascent_x * point.x + ascent_y * point.y);
    cross_values.push_back(cross_x * point.x + cross_y * point.y);
  }
  const double along_low = quantile(along_values, 0.02);
  const double along_high = quantile(along_values, 0.98);
  const double cross_low = quantile(cross_values, 0.02);
  const double cross_high = quantile(cross_values, 0.98);
  const double length = along_high - along_low;
  const double width = cross_high - cross_low;
  const double height_span = gradient * length;
  if (length < config.min_length || width < config.min_width ||
    height_span < config.min_height_span)
  {
    return std::nullopt;
  }

  const auto xy_from_projection = [ascent_x, ascent_y, cross_x, cross_y](
    double along, double across) {
      return Point2d{
      ascent_x * along + cross_x * across,
      ascent_y * along + cross_y * across};
    };
  RampRegion region;
  region.name = "automatic_ramp";
  region.polygon = {
    xy_from_projection(along_low, cross_low),
    xy_from_projection(along_high, cross_low),
    xy_from_projection(along_high, cross_high),
    xy_from_projection(along_low, cross_high),
  };
  const double cross_center = 0.5 * (cross_low + cross_high);
  const auto origin = xy_from_projection(along_low, cross_center);
  region.origin_x = origin.x;
  region.origin_y = origin.y;
  region.origin_height =
    best_plane.a * origin.x + best_plane.b * origin.y + best_plane.c;
  region.ascent_yaw_rad = std::atan2(ascent_y, ascent_x);
  region.slope_rad = slope;
  region.surface_tolerance = config.surface_tolerance;
  std::string validation_error;
  if (!validate_ramp_region(region, validation_error)) {
    return std::nullopt;
  }

  AutomaticRampDetection result;
  result.region = std::move(region);
  result.inlier_count = best_inliers.size();
  result.inlier_ratio =
    static_cast<double>(best_inliers.size()) / static_cast<double>(points.size());
  result.rms_residual = std::sqrt(squared_error / static_cast<double>(best_inliers.size()));
  result.length = length;
  result.width = width;
  result.height_span = height_span;
  return result;
}

}  // namespace rm_mid360_driver_bridge
