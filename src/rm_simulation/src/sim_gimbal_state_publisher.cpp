#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

class SimGimbalStatePublisher : public rclcpp::Node
{
public:
  SimGimbalStatePublisher()
  : Node("sim_gimbal_state_publisher")
  {
    state_topic_ = declare_parameter<std::string>("state_topic", "/gimbal/state");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/simulation/gimbal_yaw/command");
    joint_name_ = declare_parameter<std::string>("joint_name", "gimbal_yaw_joint");
    offset_rad_ = declare_parameter<double>("offset_rad", 0.65);
    amplitude_rad_ = declare_parameter<double>("amplitude_rad", 0.0);
    frequency_hz_ = declare_parameter<double>("frequency_hz", 0.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    if (publish_rate_hz_ <= 0.0 || frequency_hz_ < 0.0) {
      throw std::invalid_argument("gimbal publish rate and frequency are invalid");
    }

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(state_topic_, 10);
    command_pub_ = create_publisher<std_msgs::msg::Float64>(command_topic_, 10);
    start_time_ = now();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publish();});
  }

private:
  void publish()
  {
    const auto stamp = now();
    double elapsed = (stamp - start_time_).seconds();
    if (elapsed < 0.0) {
      start_time_ = stamp;
      elapsed = 0.0;
    }
    constexpr double kTwoPi = 6.28318530717958647692;
    const double phase = kTwoPi * frequency_hz_ * elapsed;
    const double yaw = offset_rad_ + amplitude_rad_ * std::sin(phase);
    const double velocity =
      kTwoPi * frequency_hz_ * amplitude_rad_ * std::cos(phase);

    sensor_msgs::msg::JointState state;
    state.header.stamp = stamp;
    state.name = {joint_name_};
    state.position = {yaw};
    state.velocity = {velocity};
    state_pub_->publish(state);

    std_msgs::msg::Float64 command;
    command.data = yaw;
    command_pub_->publish(command);
  }

  std::string state_topic_;
  std::string command_topic_;
  std::string joint_name_;
  double offset_rad_;
  double amplitude_rad_;
  double frequency_hz_;
  double publish_rate_hz_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimGimbalStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
