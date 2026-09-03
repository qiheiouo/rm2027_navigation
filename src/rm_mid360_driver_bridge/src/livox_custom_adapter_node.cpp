#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_mid360_driver_bridge/livox_message_conversion.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LivoxCustomAdapterNode : public rclcpp::Node
{
public:
  LivoxCustomAdapterNode()
  : Node("livox_custom_adapter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    custom_output_topic_ = declare_parameter<std::string>("custom_output_topic", "");
    pointcloud_output_topic_ = declare_parameter<std::string>(
      "pointcloud_output_topic", "");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");

    if (input_topic_.empty() || custom_output_topic_.empty() ||
      pointcloud_output_topic_.empty() || output_frame_id_.empty())
    {
      throw std::invalid_argument(
              "input_topic, custom_output_topic, pointcloud_output_topic and "
              "output_frame_id must not be empty");
    }

    custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      custom_output_topic_, rclcpp::SensorDataQoS());
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LivoxCustomAdapterNode::handle_message, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Native Livox timing path: %s -> custom=%s, PointCloud2=%s, frame=%s",
      input_topic_.c_str(), custom_output_topic_.c_str(),
      pointcloud_output_topic_.c_str(), output_frame_id_.c_str());
  }

private:
  void handle_message(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    std::string error;
    if (!rm_mid360_driver_bridge::custom_msg_to_pointcloud2(
        *message, output_frame_id_, cloud, error))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "drop malformed native Livox CustomMsg: %s", error.c_str());
      return;
    }

    // Preserve native timebase and per-point offset_time byte-for-byte for
    // FAST-LIO. Only the project-owned frame name is canonicalized.
    auto canonical = *message;
    canonical.header.frame_id = output_frame_id_;
    custom_pub_->publish(canonical);
    pointcloud_pub_->publish(cloud);
  }

  std::string input_topic_;
  std::string custom_output_topic_;
  std::string pointcloud_output_topic_;
  std::string output_frame_id_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
