#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/gimbal_state.hpp"
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
    frame_id_ = declare_parameter<std::string>("frame_id", "gimbal_yaw_link");
    motion_mode_ = declare_parameter<std::string>("motion_mode", "fixed");
    offset_rad_ = declare_parameter<double>("offset_rad", 0.65);
    amplitude_rad_ = declare_parameter<double>("amplitude_rad", 0.0);
    frequency_hz_ = declare_parameter<double>("frequency_hz", 0.0);
    angular_velocity_rad_s_ = declare_parameter<double>(
      "angular_velocity_rad_s", 0.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    if (publish_rate_hz_ <= 0.0 || frequency_hz_ < 0.0 ||
      (motion_mode_ != "fixed" && motion_mode_ != "sine" &&
      motion_mode_ != "continuous"))
    {
      throw std::invalid_argument("gimbal publish rate and frequency are invalid");
    }

    state_pub_ = create_publisher<rm_competition_interfaces::msg::GimbalState>(
      state_topic_, 10);
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
    double yaw = offset_rad_;
    double velocity = 0.0;
    if (motion_mode_ == "sine") {
      yaw += amplitude_rad_ * std::sin(phase);
      velocity = kTwoPi * frequency_hz_ * amplitude_rad_ * std::cos(phase);
    } else if (motion_mode_ == "continuous") {
      yaw += angular_velocity_rad_s_ * elapsed;
      velocity = angular_velocity_rad_s_;
    }

    rm_competition_interfaces::msg::GimbalState state;
    state.header.stamp = stamp;
    state.header.frame_id = frame_id_;
    state.relative_yaw_rad = yaw;
    state.yaw_rate_rad_s = velocity;
    state.sample_sequence = sample_sequence_++;
    constexpr double kUint32Range = 4294967296.0;
    state.mcu_time_ms = static_cast<std::uint32_t>(
      std::fmod(std::max(0.0, elapsed) * 1000.0, kUint32Range));
    state.online = true;
    state.valid = true;
    state_pub_->publish(state);

    std_msgs::msg::Float64 command;
    command.data = yaw;
    command_pub_->publish(command);
  }

  std::string state_topic_;
  std::string command_topic_;
  std::string frame_id_;
  std::string motion_mode_;
  double offset_rad_;
  double amplitude_rad_;
  double frequency_hz_;
  double angular_velocity_rad_s_;
  double publish_rate_hz_;
  std::uint32_t sample_sequence_ = 0;
  rclcpp::Time start_time_;
  rclcpp::Publisher<rm_competition_interfaces::msg::GimbalState>::SharedPtr state_pub_;
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
