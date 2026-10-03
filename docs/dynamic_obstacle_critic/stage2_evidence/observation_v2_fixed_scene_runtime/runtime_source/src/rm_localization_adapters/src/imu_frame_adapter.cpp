#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuFrameAdapter : public rclcpp::Node
{
public:
  ImuFrameAdapter()
  : Node("imu_frame_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lio_imu_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lio_imu");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "lio_imu_link");
    warn_on_frame_override_ = declare_parameter<bool>("warn_on_frame_override", true);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuFrameAdapter::handle_imu, this, std::placeholders::_1));

    RCLCPP_WARN(
      get_logger(),
      "Phase 1 IMU frame adapter: %s -> %s, output frame_id=%s. "
      "This node only rewrites header.frame_id; it does not publish TF, "
      "filter IMU data, or modify measurements.",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_id_.c_str());
  }

private:
  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto output = *msg;
    if (warn_on_frame_override_ && output.header.frame_id != output_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Rewriting IMU frame_id from '%s' to '%s'. Measurement values and stamp are unchanged.",
        output.header.frame_id.c_str(),
        output_frame_id_.c_str());
    }
    output.header.frame_id = output_frame_id_;
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool warn_on_frame_override_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFrameAdapter>());
  rclcpp::shutdown();
  return 0;
}
