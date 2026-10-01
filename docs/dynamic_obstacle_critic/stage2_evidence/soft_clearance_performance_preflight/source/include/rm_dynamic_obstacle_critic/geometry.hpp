// Extracted from frozen research e680b14 geometry.hpp; experimental
// occupancy/ranking removed.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
namespace rm_dynamic_obstacle_critic {
struct Point {
  double x, y;
};
struct Box {
  double min_x, min_y, max_x, max_y;
};
inline std::vector<Point> transform(const std::vector<Point> &polygon, double x,
                                    double y, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<Point> out;
  out.reserve(polygon.size());
  for (const auto &p : polygon)
    out.push_back({x + c * p.x - s * p.y, y + s * p.x + c * p.y});
  return out;
}
inline double segment_distance(Point a, Point b, Point p) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  const double u =
      len2 > 0
          ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0)
          : 0;
  return std::hypot(p.x - a.x - u * dx, p.y - a.y - u * dy);
}
inline double closer_segment_distance(Point a, Point b, Point p,
                                      double current) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  const double u =
      len2 > 0
          ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0)
          : 0;
  const double rx = p.x - a.x - u * dx, ry = p.y - a.y - u * dy;
  // L-infinity is a lower bound on hypot. Keep the original residual and
  // hypot arithmetic for every term which can improve the exact minimum.
  if (std::max(std::abs(rx), std::abs(ry)) >= current)
    return current;
  return std::min(current, std::hypot(rx, ry));
}
inline bool separated_on_axis(const std::vector<Point> &poly,
                              const std::vector<Point> &box, double ax,
                              double ay) {
  double pmin = std::numeric_limits<double>::infinity(), pmax = -pmin;
  double bmin = pmin, bmax = -pmin;
  for (const auto &p : poly) {
    const double v = p.x * ax + p.y * ay;
    pmin = std::min(pmin, v);
    pmax = std::max(pmax, v);
  }
  for (const auto &p : box) {
    const double v = p.x * ax + p.y * ay;
    bmin = std::min(bmin, v);
    bmax = std::max(bmax, v);
  }
  return pmax < bmin || bmax < pmin;
}
inline double polygon_box_distance(const std::vector<Point> &poly,
                                   const Box &b) {
  if (poly.size() < 3 || b.min_x > b.max_x || b.min_y > b.max_y)
    throw std::invalid_argument("invalid polygon/box");
  const std::vector<Point> box{{b.min_x, b.min_y},
                               {b.max_x, b.min_y},
                               {b.max_x, b.max_y},
                               {b.min_x, b.max_y}};
  bool separated =
      separated_on_axis(poly, box, 1, 0) || separated_on_axis(poly, box, 0, 1);
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto &a = poly[i];
    const auto &c = poly[(i + 1) % poly.size()];
    separated =
        separated || separated_on_axis(poly, box, -(c.y - a.y), c.x - a.x);
  }
  if (!separated)
    return 0;
  double distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto &a = poly[i];
    const auto &c = poly[(i + 1) % poly.size()];
    for (const auto &p : box)
      distance = closer_segment_distance(a, c, p, distance);
  }
  for (size_t i = 0; i < box.size(); ++i) {
    const auto &a = box[i];
    const auto &c = box[(i + 1) % box.size()];
    for (const auto &p : poly)
      distance = closer_segment_distance(a, c, p, distance);
  }
  return distance;
}

// A separating-axis gap divided by axis length is a lower bound on Euclidean
// distance. Reject only clearly distant cells; the original exact distance is
// still evaluated for every cell which may be within the requested reserve.
inline bool polygon_box_may_be_within(const std::vector<Point> &poly,
                                      const Box &b, double limit) {
  const std::array<Point, 4> box{{{b.min_x, b.min_y},
                                  {b.max_x, b.min_y},
                                  {b.max_x, b.max_y},
                                  {b.min_x, b.max_y}}};
  auto distant = [&](double ax, double ay) {
    double pmin = INFINITY, pmax = -INFINITY, bmin = INFINITY, bmax = -INFINITY;
    for (auto p : poly) {
      double value = p.x * ax + p.y * ay;
      pmin = std::min(pmin, value);
      pmax = std::max(pmax, value);
    }
    for (auto p : box) {
      double value = p.x * ax + p.y * ay;
      bmin = std::min(bmin, value);
      bmax = std::max(bmax, value);
    }
    const double gap = std::max(bmin - pmax, pmin - bmax);
    return gap > (limit + 1e-12) * std::hypot(ax, ay);
  };
  if (distant(1, 0) || distant(0, 1))
    return false;
  for (size_t i = 0; i < poly.size(); ++i) {
    auto a = poly[i], c = poly[(i + 1) % poly.size()];
    if (distant(-(c.y - a.y), c.x - a.x))
      return false;
  }
  return true;
}

inline bool inside(const std::vector<Point> &poly, Point p) {
  bool result = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto &a = poly[i];
    const auto &b = poly[j];
    if ((a.y > p.y) != (b.y > p.y) &&
        p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
      result = !result;
  }
  return result;
}
inline void validate_footprint(const std::vector<Point> &poly) {
  if (poly.size() < 3)
    throw std::invalid_argument("footprint needs at least 3 vertices");
  double sign = 0, area = 0;
  for (size_t i = 0; i < poly.size(); ++i) {
    auto a = poly[i], b = poly[(i + 1) % poly.size()],
         c = poly[(i + 2) % poly.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y))
      throw std::invalid_argument("nonfinite footprint");
    if (std::hypot(a.x - b.x, a.y - b.y) < 1e-9)
      throw std::invalid_argument("duplicate footprint vertex");
    double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    if (std::abs(cross) > 1e-9) {
      if (sign * cross < 0)
        throw std::invalid_argument("footprint must be convex and ordered");
      sign = cross;
    }
    area += a.x * b.y - a.y * b.x;
  }
  if (std::abs(area) < 1e-9 || sign == 0)
    throw std::invalid_argument("degenerate footprint");
}
inline double circle_clearance(const std::vector<Point> &poly, Point center,
                               double radius) {
  double d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < poly.size(); ++i)
    d = std::min(
        d, segment_distance(poly[i], poly[(i + 1) % poly.size()], center));
  return (inside(poly, center) ? -d : d) - radius;
}
struct Pose {
  double x, y, yaw;
};
struct Velocity {
  double x, y, yaw;
};
inline Pose advance(Pose p, Velocity v, double dt) {
  // Exact planar integration of a constant body-frame twist (Omni or
  // DiffDrive).
  double dx = v.x * dt, dy = v.y * dt;
  if (std::abs(v.yaw) > 1e-9) {
    const double a = v.yaw * dt;
    dx = (v.x * std::sin(a) + v.y * (std::cos(a) - 1)) / v.yaw;
    dy = (v.x * (1 - std::cos(a)) + v.y * std::sin(a)) / v.yaw;
  }
  return {p.x + std::cos(p.yaw) * dx - std::sin(p.yaw) * dy,
          p.y + std::sin(p.yaw) * dx + std::cos(p.yaw) * dy,
          p.yaw + v.yaw * dt};
}
} // namespace rm_dynamic_obstacle_critic
