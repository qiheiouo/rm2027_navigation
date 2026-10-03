#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class FakePointCloudObstaclePublisher : public rclcpp::Node
{
public:
  FakePointCloudObstaclePublisher()
  : Node("fake_pointcloud_obstacle_publisher")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/points/obstacles");
    frame_id_ = declare_parameter<std::string>("frame_id", "sim_lidar_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    obstacle_x_ = declare_parameter<double>("obstacle_x", 1.4);
    obstacle_y_ = declare_parameter<double>("obstacle_y", 0.0);
    obstacle_z_ = declare_parameter<double>("obstacle_z", 0.0);
    width_y_ = declare_parameter<double>("width_y", 0.5);
    height_z_ = declare_parameter<double>("height_z", 0.7);
    y_samples_ = std::max(2, static_cast<int>(declare_parameter<int>("y_samples", 11)));
    z_samples_ = std::max(2, static_cast<int>(declare_parameter<int>("z_samples", 8)));

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive and finite");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&FakePointCloudObstaclePublisher::publishCloud, this));

    RCLCPP_WARN(
      get_logger(),
      "Simulation-only PointCloud2 obstacle publisher: topic=%s frame_id=%s. "
      "No TF, odom, cmd_vel, or navigation goal is published.",
      output_topic_.c_str(), frame_id_.c_str());
  }

private:
  void publishCloud()
  {
    const std::size_t point_count =
      static_cast<std::size_t>(y_samples_) * static_cast<std::size_t>(z_samples_);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(point_count);
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> x_iter(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iter(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iter(cloud, "z");

    for (int zi = 0; zi < z_samples_; ++zi) {
      const double z_ratio = z_samples_ == 1 ? 0.0 : static_cast<double>(zi) / (z_samples_ - 1);
      const double z = obstacle_z_ - height_z_ * 0.5 + z_ratio * height_z_;
      for (int yi = 0; yi < y_samples_; ++yi) {
        const double y_ratio = y_samples_ == 1 ? 0.0 : static_cast<double>(yi) / (y_samples_ - 1);
        const double y = obstacle_y_ - width_y_ * 0.5 + y_ratio * width_y_;
        *x_iter = static_cast<float>(obstacle_x_);
        *y_iter = static_cast<float>(y);
        *z_iter = static_cast<float>(z);
        ++x_iter;
        ++y_iter;
        ++z_iter;
      }
    }

    publisher_->publish(cloud);
  }

  std::string output_topic_;
  std::string frame_id_;
  double publish_rate_hz_;
  double obstacle_x_;
  double obstacle_y_;
  double obstacle_z_;
  double width_y_;
  double height_z_;
  int y_samples_;
  int z_samples_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakePointCloudObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}
