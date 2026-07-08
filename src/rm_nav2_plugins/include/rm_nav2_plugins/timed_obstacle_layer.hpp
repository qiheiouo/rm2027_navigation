#ifndef RM_NAV2_PLUGINS__TIMED_OBSTACLE_LAYER_HPP_
#define RM_NAV2_PLUGINS__TIMED_OBSTACLE_LAYER_HPP_

#include <cstdint>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace rm_nav2_plugins
{

class TimedObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  TimedObstacleLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override;
  void reset() override;
  bool isClearable() override;

private:
  struct CellKey
  {
    std::int64_t x;
    std::int64_t y;

    bool operator==(const CellKey & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellKeyHash
  {
    std::size_t operator()(const CellKey & key) const
    {
      const auto hx = std::hash<std::int64_t>{}(key.x);
      const auto hy = std::hash<std::int64_t>{}(key.y);
      return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2));
    }
  };

  struct TimedCell
  {
    double wx;
    double wy;
    rclcpp::Time last_seen;
  };

  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void expireOldCells(const rclcpp::Time & now);
  CellKey makeCellKey(double wx, double wy) const;
  void touchBounds(
    const TimedCell & cell, double radius,
    double * min_x, double * min_y, double * max_x, double * max_y) const;
  void clearCellNeighborhood(
    nav2_costmap_2d::Costmap2D & master_grid,
    const TimedCell & cell,
    int min_i, int min_j, int max_i, int max_j) const;

  std::mutex mutex_;
  std::unordered_map<CellKey, TimedCell, CellKeyHash> active_cells_;
  std::unordered_map<CellKey, TimedCell, CellKeyHash> expired_cells_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Time last_scan_time_;

  std::string global_frame_;
  std::string topic_{"/local_scan"};
  double decay_time_sec_{1.0};
  double obstacle_min_range_{0.45};
  double obstacle_max_range_{3.5};
  double expected_update_rate_{0.2};
  double transform_tolerance_{0.05};
  double cell_resolution_{0.05};
  double clear_radius_{0.35};
  int combination_method_{1};
};

}  // namespace rm_nav2_plugins

#endif  // RM_NAV2_PLUGINS__TIMED_OBSTACLE_LAYER_HPP_
