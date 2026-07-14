#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PointcloudFusionNode : public rclcpp::Node
{
public:
  PointcloudFusionNode()
  : Node("pointcloud_fusion_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics",
      {"/livox/left/pointcloud_filtered", "/livox/right/pointcloud_filtered"});
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/points/obstacles_fused");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", 0.15);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    require_all_inputs_ = declare_parameter<bool>("require_all_inputs", false);
    max_output_points_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("max_output_points", 200000)));

    if (input_topics_.empty() || output_topic_.empty() || target_frame_.empty()) {
      throw std::invalid_argument("input_topics, output_topic and target_frame must be set");
    }
    if (max_cloud_age_sec_ < 0.0 || transform_timeout_sec_ < 0.0) {
      throw std::invalid_argument("fusion time limits must not be negative");
    }

    caches_.resize(input_topics_.size());
    subscriptions_.reserve(input_topics_.size());
    for (std::size_t index = 0; index < input_topics_.size(); ++index) {
      if (input_topics_[index].empty()) {
        throw std::invalid_argument("pointcloud fusion input topic must not be empty");
      }
      subscriptions_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[index], rclcpp::SensorDataQoS(),
        [this, index](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
          handleCloud(index, *message);
        }));
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(), "pointcloud fusion ready: inputs=%zu output=%s frame=%s require_all=%s",
      input_topics_.size(), output_topic_.c_str(), target_frame_.c_str(),
      require_all_inputs_ ? "true" : "false");
  }

private:
  struct CachedCloud
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::vector<std::array<float, 3>> points;
    bool valid{false};
  };

  void handleCloud(std::size_t index, const sensor_msgs::msg::PointCloud2 & message)
  {
    const rclcpp::Time stamp(message.header.stamp);
    if (stamp.nanoseconds() <= 0 || message.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "drop fusion input with zero timestamp or empty frame_id");
      return;
    }
    const double source_age = (now() - stamp).seconds();
    if (!std::isfinite(source_age) || source_age < -max_cloud_age_sec_ ||
      source_age > max_cloud_age_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "drop stale fusion input: age=%.3f s limit=%.3f s",
        source_age, max_cloud_age_sec_);
      return;
    }

    tf2::Transform input_to_target;
    if (!lookupTransform(message, input_to_target)) {
      return;
    }

    std::vector<std::array<float, 3>> transformed;
    transformed.reserve(message.width * message.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> input_x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_z(message, "z");
      for (; input_x != input_x.end(); ++input_x, ++input_y, ++input_z) {
        if (!std::isfinite(*input_x) || !std::isfinite(*input_y) || !std::isfinite(*input_z)) {
          continue;
        }
        const tf2::Vector3 point = input_to_target * tf2::Vector3(*input_x, *input_y, *input_z);
        transformed.push_back({
          static_cast<float>(point.x()),
          static_cast<float>(point.y()),
          static_cast<float>(point.z())});
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot decode fusion pointcloud fields: %s", error.what());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      caches_[index].stamp = stamp;
      caches_[index].points = std::move(transformed);
      caches_[index].valid = true;
    }
    publishFused(stamp);
  }

  bool lookupTransform(
    const sensor_msgs::msg::PointCloud2 & message,
    tf2::Transform & input_to_target)
  {
    if (message.header.frame_id == target_frame_) {
      input_to_target.setIdentity();
      return true;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, message.header.frame_id, message.header.stamp,
        tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(transform.transform, input_to_target);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot transform fusion input %s -> %s: %s",
        message.header.frame_id.c_str(), target_frame_.c_str(), error.what());
      return false;
    }
  }

  void publishFused(const rclcpp::Time & reference_stamp)
  {
    std::vector<std::array<float, 3>> points;
    std::size_t included_inputs = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & cache : caches_) {
        if (!cache.valid) {
          continue;
        }
        const double age = std::abs((reference_stamp - cache.stamp).seconds());
        if (age > max_cloud_age_sec_) {
          continue;
        }
        ++included_inputs;
        const std::size_t remaining = max_output_points_ - points.size();
        const std::size_t count = std::min(remaining, cache.points.size());
        points.insert(points.end(), cache.points.begin(), cache.points.begin() + count);
        if (points.size() >= max_output_points_) {
          break;
        }
      }
    }

    if (included_inputs == 0 || (require_all_inputs_ && included_inputs != caches_.size())) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp = reference_stamp;
    output.header.frame_id = target_frame_;
    output.height = 1;
    output.is_bigendian = false;
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    for (const auto & point : points) {
      *output_x = point[0];
      *output_y = point[1];
      *output_z = point[2];
      ++output_x;
      ++output_y;
      ++output_z;
    }
    publisher_->publish(output);
  }

  std::vector<std::string> input_topics_;
  std::string output_topic_;
  std::string target_frame_;
  double max_cloud_age_sec_;
  double transform_timeout_sec_;
  bool require_all_inputs_;
  std::size_t max_output_points_;

  std::mutex mutex_;
  std::vector<CachedCloud> caches_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
  subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
