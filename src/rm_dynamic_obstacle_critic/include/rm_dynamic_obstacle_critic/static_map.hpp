#pragma once
#include "nav2_msgs/msg/costmap.hpp"
#include "rm_dynamic_obstacle_critic/geometry.hpp"
#include <string>
namespace rm_dynamic_obstacle_critic {
struct StaticMapCheck {
  bool clear{false};
  std::string reason{"invalid_map_geometry"};
  int cell_x{-1}, cell_y{-1};
  unsigned int cost{0};
  Point cell_center{}; // In the costmap header frame.
  double distance{std::numeric_limits<double>::infinity()};
};
inline StaticMapCheck check_static_map(const nav2_msgs::msg::Costmap &map,
                                       const std::vector<Point> &world,
                                       double reserve,
                                       unsigned int collision_threshold = 203) {
  StaticMapCheck result;
  const auto &m = map.metadata;
  const auto &q = m.origin.orientation;
  if (m.size_x == 0 || m.size_y == 0 || m.resolution <= 0 ||
      !std::isfinite(m.resolution) ||
      map.data.size() != static_cast<size_t>(m.size_x) * m.size_y ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || std::abs(q.x) > 1e-3 || std::abs(q.y) > 1e-3 ||
      std::abs(q.z * q.z + q.w * q.w - 1) > 1e-3 ||
      !std::isfinite(m.origin.position.x) ||
      !std::isfinite(m.origin.position.y))
    return result;
  if (!std::isfinite(reserve) || reserve < 0 || world.size() < 3) {
    result.reason = "invalid_footprint_or_reserve";
    return result;
  }
  double yaw = std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z);
  std::vector<Point> poly;
  double minx = INFINITY, miny = INFINITY, maxx = -INFINITY, maxy = -INFINITY;
  for (auto p : world) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      result.reason = "invalid_footprint_or_reserve";
      return result;
    }
    const double dx = p.x - m.origin.position.x, dy = p.y - m.origin.position.y;
    Point local{std::cos(yaw) * dx + std::sin(yaw) * dy,
                -std::sin(yaw) * dx + std::cos(yaw) * dy};
    poly.push_back(local);
    minx = std::min(minx, local.x);
    maxx = std::max(maxx, local.x);
    miny = std::min(miny, local.y);
    maxy = std::max(maxy, local.y);
  }
  if (minx - reserve < 0 || miny - reserve < 0 ||
      maxx + reserve >= m.size_x * m.resolution ||
      maxy + reserve >= m.size_y * m.resolution) {
    result.reason = "outside_map";
    return result;
  }
  int x0 = std::floor((minx - reserve) / m.resolution),
      y0 = std::floor((miny - reserve) / m.resolution);
  int x1 = std::floor((maxx + reserve) / m.resolution),
      y1 = std::floor((maxy + reserve) / m.resolution);
  if (x0 < 0 || y0 < 0 || x1 >= static_cast<int>(m.size_x) ||
      y1 >= static_cast<int>(m.size_y)) {
    result.reason = "outside_map";
    return result;
  }
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x) {
      if (map.data[y * m.size_x + x] < collision_threshold)
        continue; // 255 unknown fails closed too.
      const Box cell{x * m.resolution, y * m.resolution, (x + 1) * m.resolution,
                     (y + 1) * m.resolution};
      if (!polygon_box_may_be_within(poly, cell, reserve + 1e-9))
        continue;
      const double gap = polygon_box_distance(poly, cell);
      if (gap <= reserve + 1e-9) {
        result.reason = map.data[y * m.size_x + x] == 255 ? "unknown_cell"
                                                          : "occupied_cell";
        result.cell_x = x;
        result.cell_y = y;
        result.cost = map.data[y * m.size_x + x];
        const double cx = (x + 0.5) * m.resolution,
                     cy = (y + 0.5) * m.resolution;
        result.cell_center = {
            m.origin.position.x + std::cos(yaw) * cx - std::sin(yaw) * cy,
            m.origin.position.y + std::sin(yaw) * cx + std::cos(yaw) * cy};
        result.distance = gap;
        return result;
      }
    }
  result.clear = true;
  result.reason = "clear";
  return result;
}
inline bool static_map_clear(const nav2_msgs::msg::Costmap &map,
                             const std::vector<Point> &world, double reserve,
                             unsigned int collision_threshold = 203) {
  return check_static_map(map, world, reserve, collision_threshold).clear;
}
} // namespace rm_dynamic_obstacle_critic
