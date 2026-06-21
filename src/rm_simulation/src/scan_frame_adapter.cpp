#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFrameAdapter : public rclcpp::Node
{
public:
  ScanFrameAdapter()
  : Node("scan_frame_adapter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/simulation/scan_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    output_frame_ = declare_parameter<std::string>("output_frame", "sim_lidar_link");

    const auto qos = rclcpp::SensorDataQoS();
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, qos,
      std::bind(&ScanFrameAdapter::handleScan, this, std::placeholders::_1));

    RCLCPP_WARN(
      get_logger(),
      "Simulation-only scan adapter: %s -> %s with frame_id=%s. No TF is published.",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str());
  }

private:
  void handleScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!logged_input_frame_) {
      RCLCPP_INFO(
        get_logger(), "Gazebo raw scan frame_id is '%s'; canonical simulation frame is '%s'.",
        msg->header.frame_id.c_str(), output_frame_.c_str());
      logged_input_frame_ = true;
    }

    sensor_msgs::msg::LaserScan output = *msg;
    output.header.frame_id = output_frame_;
    scan_pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  bool logged_input_frame_ = false;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFrameAdapter>());
  rclcpp::shutdown();
  return 0;
}
