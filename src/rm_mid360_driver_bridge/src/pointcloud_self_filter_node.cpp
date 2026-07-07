#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

struct CropBox
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
};

bool inside_box(const tf2::Vector3 & point, const CropBox & box)
{
  return point.x() >= box.min_x && point.x() <= box.max_x &&
         point.y() >= box.min_y && point.y() <= box.max_y &&
         point.z() >= box.min_z && point.z() <= box.max_z;
}

}  // namespace

class PointcloudSelfFilterNode : public rclcpp::Node
{
public:
  PointcloudSelfFilterNode()
  : Node("pointcloud_self_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/left/pointcloud");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/livox/left/pointcloud_filtered");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    filter_enabled_ = declare_parameter<bool>("filter_enabled", true);
    remove_self_box_ = declare_parameter<bool>("remove_self_box", true);
    remove_below_z_ = declare_parameter<bool>("remove_below_z", false);
    remove_above_z_ = declare_parameter<bool>("remove_above_z", false);
    min_z_ = declare_parameter<double>("min_z", 0.05);
    max_z_ = declare_parameter<double>("max_z", 2.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    publish_unfiltered_on_tf_failure_ =
      declare_parameter<bool>("publish_unfiltered_on_tf_failure", false);

    self_box_.min_x = declare_parameter<double>("self_box.min_x", -0.45);
    self_box_.max_x = declare_parameter<double>("self_box.max_x", 0.45);
    self_box_.min_y = declare_parameter<double>("self_box.min_y", -0.40);
    self_box_.max_y = declare_parameter<double>("self_box.max_y", 0.40);
    self_box_.min_z = declare_parameter<double>("self_box.min_z", -0.25);
    self_box_.max_z = declare_parameter<double>("self_box.max_z", 0.80);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudSelfFilterNode::cloud_callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "pointcloud self filter: %s -> %s, target_frame=%s, enabled=%s",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      filter_enabled_ ? "true" : "false");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!filter_enabled_) {
      pub_->publish(*msg);
      return;
    }

    tf2::Transform input_to_target;
    if (!lookup_input_to_target(*msg, input_to_target)) {
      if (publish_unfiltered_on_tf_failure_) {
        pub_->publish(*msg);
      }
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> input_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> input_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> input_z(*msg, "z");

    sensor_msgs::msg::PointCloud2 output;
    output.header = msg->header;
    output.height = 1;
    output.is_bigendian = msg->is_bigendian;
    output.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(msg->width * msg->height);

    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");

    std::size_t kept = 0;
    std::size_t removed = 0;
    for (; input_x != input_x.end(); ++input_x, ++input_y, ++input_z) {
      const float x = *input_x;
      const float y = *input_y;
      const float z = *input_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++removed;
        continue;
      }

      const tf2::Vector3 point_in_input(x, y, z);
      const tf2::Vector3 point_in_target = input_to_target * point_in_input;

      if (should_remove(point_in_target)) {
        ++removed;
        continue;
      }

      *output_x = x;
      *output_y = y;
      *output_z = z;
      ++output_x;
      ++output_y;
      ++output_z;
      ++kept;
    }

    modifier.resize(kept);
    output.width = static_cast<std::uint32_t>(kept);

    RCLCPP_DEBUG(
      get_logger(), "filtered cloud kept=%zu removed=%zu frame=%s",
      kept, removed, msg->header.frame_id.c_str());
    pub_->publish(output);
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

    if (msg.header.frame_id == target_frame_) {
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

  bool should_remove(const tf2::Vector3 & point_in_target) const
  {
    if (remove_self_box_ && inside_box(point_in_target, self_box_)) {
      return true;
    }
    if (remove_below_z_ && point_in_target.z() < min_z_) {
      return true;
    }
    if (remove_above_z_ && point_in_target.z() > max_z_) {
      return true;
    }
    return false;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  bool filter_enabled_;
  bool remove_self_box_;
  bool remove_below_z_;
  bool remove_above_z_;
  bool publish_unfiltered_on_tf_failure_;
  double min_z_;
  double max_z_;
  double transform_timeout_sec_;
  CropBox self_box_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudSelfFilterNode>());
  rclcpp::shutdown();
  return 0;
}
