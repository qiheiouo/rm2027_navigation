#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class Fake2DLocalizationWorld : public rclcpp::Node
{
public:
  Fake2DLocalizationWorld()
  : Node("fake_2d_localization_world")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    scan_frame_ = declare_parameter<std::string>("scan_frame", "base_link");
    resolution_ = declare_parameter<double>("resolution", 0.1);
    width_ = declare_parameter<int>("width", 100);
    height_ = declare_parameter<int>("height", 100);
    scan_rate_hz_ = declare_parameter<double>("scan_rate_hz", 10.0);
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 8.0);
    robot_x_ = declare_parameter<double>("robot_x", 0.0);
    robot_y_ = declare_parameter<double>("robot_y", 0.0);
    robot_yaw_ = declare_parameter<double>("robot_yaw", 0.0);

    if (
      resolution_ <= 0.0 || width_ < 20 || height_ < 20 || scan_rate_hz_ <= 0.0 ||
      range_min_ < 0.0 || range_max_ <= range_min_)
    {
      throw std::invalid_argument("invalid fake 2D world parameters");
    }

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/localization/scan", rclcpp::SensorDataQoS());
    buildMap();
    map_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publishMap();});
    scan_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / scan_rate_hz_)),
      [this]() {publishScan();});
    publishMap();
  }

private:
  void setOccupied(const int x, const int y)
  {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
      map_.data[static_cast<std::size_t>(y * width_ + x)] = 100;
    }
  }

  void buildMap()
  {
    map_.header.frame_id = map_frame_;
    map_.info.resolution = static_cast<float>(resolution_);
    map_.info.width = static_cast<std::uint32_t>(width_);
    map_.info.height = static_cast<std::uint32_t>(height_);
    map_.info.origin.position.x = -0.5 * static_cast<double>(width_) * resolution_;
    map_.info.origin.position.y = -0.5 * static_cast<double>(height_) * resolution_;
    map_.info.origin.orientation.w = 1.0;
    map_.data.assign(static_cast<std::size_t>(width_ * height_), 0);

    for (int x = 0; x < width_; ++x) {
      setOccupied(x, 0);
      setOccupied(x, 1);
      setOccupied(x, height_ - 1);
      setOccupied(x, height_ - 2);
    }
    for (int y = 0; y < height_; ++y) {
      setOccupied(0, y);
      setOccupied(1, y);
      setOccupied(width_ - 1, y);
      setOccupied(width_ - 2, y);
    }

    const int wall_x = worldToGridX(2.0);
    for (double y = -1.0; y <= 2.0; y += resolution_) {
      setOccupied(wall_x, worldToGridY(y));
    }
    const int shelf_y = worldToGridY(-2.2);
    for (double x = -3.0; x <= -1.0; x += resolution_) {
      setOccupied(worldToGridX(x), shelf_y);
    }
  }

  int worldToGridX(const double x) const
  {
    return static_cast<int>(std::floor((x - map_.info.origin.position.x) / resolution_));
  }

  int worldToGridY(const double y) const
  {
    return static_cast<int>(std::floor((y - map_.info.origin.position.y) / resolution_));
  }

  bool occupiedAt(const double x, const double y) const
  {
    const int grid_x = worldToGridX(x);
    const int grid_y = worldToGridY(y);
    if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
      return true;
    }
    return map_.data[static_cast<std::size_t>(grid_y * width_ + grid_x)] >= 65;
  }

  float raycast(const double angle) const
  {
    const double step = 0.5 * resolution_;
    for (double range = range_min_; range <= range_max_; range += step) {
      const double x = robot_x_ + range * std::cos(angle);
      const double y = robot_y_ + range * std::sin(angle);
      if (occupiedAt(x, y)) {
        return static_cast<float>(range);
      }
    }
    return std::numeric_limits<float>::infinity();
  }

  void publishMap()
  {
    map_.header.stamp = now();
    map_pub_->publish(map_);
  }

  void publishScan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = scan_frame_;
    scan.angle_min = static_cast<float>(-kPi);
    scan.angle_max = static_cast<float>(kPi);
    scan.angle_increment = static_cast<float>(kPi / 180.0);
    scan.time_increment = 0.0F;
    scan.scan_time = static_cast<float>(1.0 / scan_rate_hz_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    const std::size_t count = static_cast<std::size_t>(
      std::floor((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1U;
    scan.ranges.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const double relative_angle =
        static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      scan.ranges.push_back(raycast(robot_yaw_ + relative_angle));
    }
    scan_pub_->publish(scan);
  }

  std::string map_frame_;
  std::string scan_frame_;
  double resolution_{0.1};
  int width_{100};
  int height_{100};
  double scan_rate_hz_{10.0};
  double range_min_{0.05};
  double range_max_{8.0};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fake2DLocalizationWorld>());
  rclcpp::shutdown();
  return 0;
}
