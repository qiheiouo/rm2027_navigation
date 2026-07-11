#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PointcloudToLaserscanNode : public rclcpp::Node
{
public:
  PointcloudToLaserscanNode()
  : Node("pointcloud_to_laserscan_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/livox/left/pointcloud_filtered");
    output_topic_ = declare_parameter<std::string>("output_topic", "/local_scan");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    constexpr double kPi = 3.14159265358979323846;
    angle_min_ = declare_parameter<double>("angle_min", -kPi);
    angle_max_ = declare_parameter<double>("angle_max", kPi);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.01);
    scan_time_ = declare_parameter<double>("scan_time", 0.066);
    range_min_ = declare_parameter<double>("range_min", 0.45);
    range_max_ = declare_parameter<double>("range_max", 6.0);
    min_height_ = declare_parameter<double>("min_height", 0.18);
    max_height_ = declare_parameter<double>("max_height", 2.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 0.0);

    if (!std::isfinite(angle_min_) || !std::isfinite(angle_max_) ||
      !std::isfinite(angle_increment_) || angle_increment_ <= 0.0 ||
      angle_max_ <= angle_min_ || !std::isfinite(range_min_) ||
      !std::isfinite(range_max_) || range_max_ <= range_min_ ||
      !std::isfinite(min_height_) || !std::isfinite(max_height_) ||
      max_height_ < min_height_ || !std::isfinite(max_publish_rate_hz_) ||
      max_publish_rate_hz_ < 0.0)
    {
      throw std::invalid_argument("invalid pointcloud_to_laserscan projection parameters");
    }

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudToLaserscanNode::cloud_callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "pointcloud to laserscan: %s -> %s, target_frame=%s, range=[%.2f, %.2f], "
      "height=[%.2f, %.2f], max_rate=%.2f Hz (0 means unlimited)",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      range_min_, range_max_, min_height_, max_height_, max_publish_rate_hz_);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto callback_time = std::chrono::steady_clock::now();
    if (max_publish_rate_hz_ > 0.0 && has_published_) {
      const double elapsed = std::chrono::duration<double>(
        callback_time - last_publish_time_).count();
      if (elapsed < 1.0 / max_publish_rate_hz_) {
        return;
      }
    }

    tf2::Transform input_to_target;
    if (!lookup_input_to_target(*msg, input_to_target)) {
      return;
    }

    const auto bin_count = static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_) + 1.0);
    std::vector<float> ranges(
      bin_count, std::numeric_limits<float>::infinity());

    sensor_msgs::PointCloud2ConstIterator<float> input_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> input_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> input_z(*msg, "z");

    std::size_t used_points = 0;
    for (; input_x != input_x.end(); ++input_x, ++input_y, ++input_z) {
      const float x = *input_x;
      const float y = *input_y;
      const float z = *input_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const tf2::Vector3 point_in_target = input_to_target * tf2::Vector3(x, y, z);
      if (point_in_target.z() < min_height_ || point_in_target.z() > max_height_) {
        continue;
      }

      const double range = std::hypot(point_in_target.x(), point_in_target.y());
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const double angle = std::atan2(point_in_target.y(), point_in_target.x());
      if (angle < angle_min_ || angle > angle_max_) {
        continue;
      }

      const auto index = static_cast<std::size_t>(
        std::floor((angle - angle_min_) / angle_increment_));
      if (index >= ranges.size()) {
        continue;
      }
      ranges[index] = std::min(ranges[index], static_cast<float>(range));
      ++used_points;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header = msg->header;
    if (!target_frame_.empty()) {
      scan.header.frame_id = target_frame_;
    }
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0F;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = std::move(ranges);

    RCLCPP_DEBUG(
      get_logger(), "projected cloud width=%u used_points=%zu bins=%zu frame=%s",
      msg->width, used_points, scan.ranges.size(), scan.header.frame_id.c_str());
    pub_->publish(scan);
    last_publish_time_ = callback_time;
    has_published_ = true;
  }

  bool lookup_input_to_target(
    const sensor_msgs::msg::PointCloud2 & msg,
    tf2::Transform & input_to_target)
  {
    if (msg.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "drop pointcloud with empty frame_id");
      return false;
    }

    if (target_frame_.empty() || msg.header.frame_id == target_frame_) {
      input_to_target.setIdentity();
      return true;
    }

    try {
      const auto timeout = tf2::durationFromSec(std::max(0.0, transform_timeout_sec_));
      const auto transform_msg = tf_buffer_.lookupTransform(
        target_frame_, msg.header.frame_id, msg.header.stamp, timeout);
      tf2::fromMsg(transform_msg.transform, input_to_target);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot transform pointcloud %s -> %s: %s",
        msg.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  double min_height_;
  double max_height_;
  double transform_timeout_sec_;
  double max_publish_rate_hz_;
  bool has_published_{false};
  std::chrono::steady_clock::time_point last_publish_time_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudToLaserscanNode>());
  rclcpp::shutdown();
  return 0;
}
