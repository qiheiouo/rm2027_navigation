#include "rm_nav2_plugins/timed_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace rm_nav2_plugins
{

void TimedObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TimedObstacleLayer failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("decay_time_sec", rclcpp::ParameterValue(decay_time_sec_));
  declareParameter("obstacle_min_range", rclcpp::ParameterValue(obstacle_min_range_));
  declareParameter("obstacle_max_range", rclcpp::ParameterValue(obstacle_max_range_));
  declareParameter("expected_update_rate", rclcpp::ParameterValue(expected_update_rate_));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  declareParameter("clear_radius", rclcpp::ParameterValue(clear_radius_));
  declareParameter("combination_method", rclcpp::ParameterValue(combination_method_));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".decay_time_sec", decay_time_sec_);
  node->get_parameter(name_ + ".obstacle_min_range", obstacle_min_range_);
  node->get_parameter(name_ + ".obstacle_max_range", obstacle_max_range_);
  node->get_parameter(name_ + ".expected_update_rate", expected_update_rate_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".clear_radius", clear_radius_);
  node->get_parameter(name_ + ".combination_method", combination_method_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  cell_resolution_ = layered_costmap_->getCostmap()->getResolution();
  clear_radius_ = std::max(clear_radius_, cell_resolution_);
  last_scan_time_ = node->now();

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&TimedObstacleLayer::scanCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "TimedObstacleLayer '%s' subscribed to %s with decay %.3fs",
    name_.c_str(), topic_.c_str(), decay_time_sec_);
}

void TimedObstacleLayer::matchSize()
{
  cell_resolution_ = layered_costmap_->getCostmap()->getResolution();
}

void TimedObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_cells_.clear();
  expired_cells_.clear();
  current_ = true;
}

bool TimedObstacleLayer::isClearable()
{
  return true;
}

void TimedObstacleLayer::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node || msg->ranges.empty() || msg->angle_increment == 0.0F) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      global_frame_, msg->header.frame_id, msg->header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "TimedObstacleLayer '%s' cannot transform %s -> %s: %s",
      name_.c_str(), msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return;
  }

  const rclcpp::Time now = node->now();
  std::unordered_map<CellKey, TimedCell, CellKeyHash> observed;
  double angle = static_cast<double>(msg->angle_min);

  geometry_msgs::msg::PointStamped scan_point;
  scan_point.header = msg->header;
  scan_point.point.z = 0.0;

  for (const auto range_value : msg->ranges) {
    const double range = static_cast<double>(range_value);
    if (std::isfinite(range) &&
      range >= obstacle_min_range_ && range <= obstacle_max_range_)
    {
      scan_point.point.x = range * std::cos(angle);
      scan_point.point.y = range * std::sin(angle);

      geometry_msgs::msg::PointStamped global_point;
      tf2::doTransform(scan_point, global_point, transform);

      TimedCell cell;
      cell.wx = global_point.point.x;
      cell.wy = global_point.point.y;
      cell.last_seen = now;
      observed[makeCellKey(cell.wx, cell.wy)] = cell;
    }
    angle += static_cast<double>(msg->angle_increment);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : observed) {
      active_cells_[item.first] = item.second;
      expired_cells_.erase(item.first);
    }
    last_scan_time_ = now;
    current_ = true;
  }
}

void TimedObstacleLayer::expireOldCells(const rclcpp::Time & now)
{
  for (auto it = active_cells_.begin(); it != active_cells_.end(); ) {
    if ((now - it->second.last_seen).seconds() > decay_time_sec_) {
      expired_cells_[it->first] = it->second;
      it = active_cells_.erase(it);
    } else {
      ++it;
    }
  }
}

void TimedObstacleLayer::updateBounds(
  double /* robot_x */, double /* robot_y */, double /* robot_yaw */,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  const rclcpp::Time now = node ? node->now() : rclcpp::Clock().now();

  std::lock_guard<std::mutex> lock(mutex_);
  expireOldCells(now);

  const double active_radius = cell_resolution_;
  for (const auto & item : active_cells_) {
    touchBounds(item.second, active_radius, min_x, min_y, max_x, max_y);
  }

  for (const auto & item : expired_cells_) {
    touchBounds(item.second, clear_radius_, min_x, min_y, max_x, max_y);
  }
}

void TimedObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & item : expired_cells_) {
    clearCellNeighborhood(master_grid, item.second, min_i, min_j, max_i, max_j);
  }
  expired_cells_.clear();

  for (const auto & item : active_cells_) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!master_grid.worldToMap(item.second.wx, item.second.wy, mx, my)) {
      continue;
    }
    if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
      static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }

    if (combination_method_ == 0 ||
      master_grid.getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION)
    {
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

TimedObstacleLayer::CellKey TimedObstacleLayer::makeCellKey(double wx, double wy) const
{
  const double resolution = std::max(cell_resolution_, 1.0e-3);
  return CellKey{
    static_cast<std::int64_t>(std::floor(wx / resolution)),
    static_cast<std::int64_t>(std::floor(wy / resolution))};
}

void TimedObstacleLayer::touchBounds(
  const TimedCell & cell, double radius,
  double * min_x, double * min_y, double * max_x, double * max_y) const
{
  *min_x = std::min(*min_x, cell.wx - radius);
  *min_y = std::min(*min_y, cell.wy - radius);
  *max_x = std::max(*max_x, cell.wx + radius);
  *max_y = std::max(*max_y, cell.wy + radius);
}

void TimedObstacleLayer::clearCellNeighborhood(
  nav2_costmap_2d::Costmap2D & master_grid,
  const TimedCell & cell,
  int min_i, int min_j, int max_i, int max_j) const
{
  unsigned int center_mx = 0;
  unsigned int center_my = 0;
  if (!master_grid.worldToMap(cell.wx, cell.wy, center_mx, center_my)) {
    return;
  }

  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(clear_radius_ / std::max(cell_resolution_, 1.0e-3))));
  const int center_i = static_cast<int>(center_mx);
  const int center_j = static_cast<int>(center_my);

  for (int my = center_j - radius_cells; my <= center_j + radius_cells; ++my) {
    if (my < min_j || my >= max_j || my < 0) {
      continue;
    }
    for (int mx = center_i - radius_cells; mx <= center_i + radius_cells; ++mx) {
      if (mx < min_i || mx >= max_i || mx < 0) {
        continue;
      }

      const double dx = (mx - center_i) * cell_resolution_;
      const double dy = (my - center_j) * cell_resolution_;
      if ((dx * dx + dy * dy) > clear_radius_ * clear_radius_) {
        continue;
      }
      master_grid.setCost(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
        nav2_costmap_2d::FREE_SPACE);
    }
  }
}

}  // namespace rm_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rm_nav2_plugins::TimedObstacleLayer, nav2_costmap_2d::Layer)
