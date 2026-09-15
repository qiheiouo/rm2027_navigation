// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#include "rm_tdt_planner/planner.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace rm_tdt_planner
{
class TdtGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    node_ = parent.lock();
    if (!node_ || !costmap) {throw std::runtime_error("missing planner node or costmap");}
    costmap_ = std::move(costmap);
    name_ = std::move(name);
    auto real = [&](const std::string & key, double value) {
        nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + key,
          rclcpp::ParameterValue(value));
        return node_->get_parameter(name_ + "." + key).as_double();
      };
    auto integer = [&](const std::string & key, int value) {
        nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + key,
          rclcpp::ParameterValue(value));
        return static_cast<int>(node_->get_parameter(name_ + "." + key).as_int());
      };
    options_.clearance = real("clearance", options_.clearance);
    options_.potential_weight = real("potential_weight", options_.potential_weight);
    options_.simplify_tolerance = real("simplify_tolerance", options_.simplify_tolerance);
    options_.output_spacing = real("output_spacing", options_.output_spacing);
    options_.time_budget = real("time_budget", options_.time_budget);
    options_.nominal_speed = real("nominal_speed", options_.nominal_speed);
    options_.nominal_acceleration = real("nominal_acceleration", options_.nominal_acceleration);
    options_.corridor_range = real("corridor_range", options_.corridor_range);
    options_.collision_iterations = integer("collision_iterations", options_.collision_iterations);
    options_.max_waypoints = integer("max_waypoints", options_.max_waypoints);
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".optimize",
      rclcpp::ParameterValue(true));
    options_.optimize = node_->get_parameter(name_ + ".optimize").as_bool();
    active_ = false;
  }
  void cleanup() override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    active_ = false; costmap_.reset(); node_.reset(); options_ = Options{};
  }
  void activate() override {std::lock_guard<std::mutex> guard(mutex_); active_ = true;}
  void deactivate() override {std::lock_guard<std::mutex> guard(mutex_); active_ = false;}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!active_ || !node_ || !costmap_) {throw nav2_core::PlannerException("inactive TDT planner");}
    const std::string frame = costmap_->getGlobalFrameID();
    if (start.header.frame_id != frame || goal.header.frame_id != frame ||
      !valid_pose(start) || !valid_pose(goal))
    {
      throw nav2_core::PlannerException("TDT requires finite poses in the global costmap frame");
    }
    auto options = options_;
    options.radius = footprint_radius();
    const Grid grid = snapshot();
    const auto result = plan(grid, {start.pose.position.x, start.pose.position.y},
      {goal.pose.position.x, goal.pose.position.y}, options);
    if (!result.success) {
      // Diagnose the exact request/snapshot; published OccupancyGrid samples may lag it.
      auto yaw = [](const geometry_msgs::msg::Quaternion & q) {
          return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        };
      std::ostringstream detail;
      detail << std::setprecision(17) << result.reason
             << " [input=nav2_master start=(" << start.pose.position.x << ',' << start.pose.position.y
             << ") goal=(" << goal.pose.position.x << ',' << goal.pose.position.y
             << ") origin=(" << grid.origin_x << ',' << grid.origin_y
             << ") size=" << grid.width << 'x' << grid.height << " resolution=" << grid.resolution
             << " radius=" << options.radius << " clearance=" << options.clearance
             << " start_yaw=" << yaw(start.pose.orientation)
             << " goal_yaw=" << yaw(goal.pose.orientation) << ']';
      throw nav2_core::PlannerException(detail.str());
    }

    // Costmap updates continue while the expensive solver runs. A changed map
    // invalidates this candidate; planner_server/BT may request a fresh plan.
    const Grid latest = snapshot();
    if (grid.width != latest.width || grid.height != latest.height ||
      grid.resolution != latest.resolution || grid.origin_x != latest.origin_x ||
      grid.origin_y != latest.origin_y || grid.costs != latest.costs ||
      options.radius != footprint_radius())
    {
      throw nav2_core::PlannerException("costmap or footprint changed during TDT planning");
    }
    nav_msgs::msg::Path output;
    output.header.frame_id = frame;
    output.header.stamp = node_->now();
    for (size_t i = 0; i < result.path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = output.header;
      pose.pose.position.x = result.path[i].x;
      pose.pose.position.y = result.path[i].y;
      double yaw = 0.0;
      if (i + 1 < result.path.size()) {
        yaw = std::atan2(result.path[i + 1].y - result.path[i].y,
          result.path[i + 1].x - result.path[i].x);
      }
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      output.poses.push_back(pose);
    }
    output.poses.front().pose.orientation = start.pose.orientation;
    output.poses.back().pose.orientation = goal.pose.orientation;
    RCLCPP_DEBUG(node_->get_logger(), "%s: %s, %.3f s", name_.c_str(),
      result.reason.c_str(), result.elapsed_seconds);
    return output;
  }

private:
  static bool valid_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
  }
  double footprint_radius() const
  {
    const auto footprint = costmap_->getRobotFootprint();
    if (footprint.size() < 3) {throw nav2_core::PlannerException("missing padded footprint");}
    double radius = 0.0;
    for (const auto & p : footprint) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        throw nav2_core::PlannerException("invalid footprint");
      }
      radius = std::max(radius, std::hypot(p.x, p.y));
    }
    return radius;
  }
  Grid snapshot() const
  {
    auto * map = costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    Grid grid;
    grid.cost_interpretation = CostInterpretation::Nav2Master;
    grid.width = map->getSizeInCellsX(); grid.height = map->getSizeInCellsY();
    grid.resolution = map->getResolution();
    grid.origin_x = map->getOriginX(); grid.origin_y = map->getOriginY();
    const size_t count = static_cast<size_t>(grid.width) * grid.height;
    if (count > 1000000 || count == 0) {throw nav2_core::PlannerException("invalid map size");}
    grid.costs.assign(map->getCharMap(), map->getCharMap() + count);
    return grid;
  }
  std::mutex mutex_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::string name_;
  Options options_;
  bool active_ = false;
};
}  // namespace rm_tdt_planner
PLUGINLIB_EXPORT_CLASS(rm_tdt_planner::TdtGlobalPlanner, nav2_core::GlobalPlanner)
