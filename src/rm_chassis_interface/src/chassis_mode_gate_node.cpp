#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/chassis_mode.hpp"
#include "std_msgs/msg/bool.hpp"

class ChassisModeGateNode : public rclcpp::Node
{
public:
  ChassisModeGateNode()
  : Node("chassis_mode_gate")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/chassis/mode_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/chassis/mode");
    valid_topic_ = declare_parameter<std::string>("valid_topic", "/chassis/mode_valid");
    max_source_age_sec_ = declare_parameter<double>("max_source_age_sec", 0.5);
    state_timeout_sec_ = declare_parameter<double>("state_timeout_sec", 0.5);
    if (max_source_age_sec_ < 0.0 || state_timeout_sec_ < 0.0) {
      throw std::invalid_argument("chassis mode time limits must not be negative");
    }

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    state_pub_ = create_publisher<rm_competition_interfaces::msg::ChassisMode>(
      output_topic_, latched);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, latched);
    subscription_ = create_subscription<rm_competition_interfaces::msg::ChassisMode>(
      input_topic_, rclcpp::QoS(10).reliable(),
      [this](rm_competition_interfaces::msg::ChassisMode::SharedPtr message) {
        handleMode(*message);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {checkTimeout();});
    publishInvalid("no state received");
    RCLCPP_INFO(
      get_logger(), "chassis authority gate ready: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  bool isConsistent(const rm_competition_interfaces::msg::ChassisMode & message) const
  {
    using Mode = rm_competition_interfaces::msg::ChassisMode;
    if (message.mode > Mode::MODE_ESTOP) {
      return false;
    }
    if (message.autonomous_enabled != (message.mode == Mode::MODE_AUTONOMOUS)) {
      return false;
    }
    if (message.emergency_stop != (message.mode == Mode::MODE_ESTOP)) {
      return false;
    }
    return true;
  }

  void handleMode(const rm_competition_interfaces::msg::ChassisMode & message)
  {
    const rclcpp::Time stamp(message.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      publishInvalid("zero timestamp");
      return;
    }
    const double age = (now() - stamp).seconds();
    if (age < -max_source_age_sec_ || age > max_source_age_sec_) {
      publishInvalid("source timestamp outside age limit");
      return;
    }
    if (!isConsistent(message)) {
      publishInvalid("inconsistent mode flags");
      return;
    }

    last_receive_time_ = now();
    received_ = true;
    state_pub_->publish(message);
    publishValid(message.online);
  }

  void checkTimeout()
  {
    if (received_ && (now() - last_receive_time_).seconds() > state_timeout_sec_) {
      received_ = false;
      publishInvalid("state timeout");
    }
  }

  void publishInvalid(const std::string & reason)
  {
    rm_competition_interfaces::msg::ChassisMode state;
    state.header.stamp = now();
    state.online = false;
    state.autonomous_enabled = false;
    state.emergency_stop = true;
    state.mode = rm_competition_interfaces::msg::ChassisMode::MODE_ESTOP;
    state_pub_->publish(state);
    publishValid(false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "chassis authority invalid: %s", reason.c_str());
  }

  void publishValid(bool value)
  {
    if (last_valid_.has_value() && *last_valid_ == value) {
      return;
    }
    last_valid_ = value;
    std_msgs::msg::Bool message;
    message.data = value;
    valid_pub_->publish(message);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string valid_topic_;
  double max_source_age_sec_;
  double state_timeout_sec_;
  bool received_{false};
  std::optional<bool> last_valid_;
  rclcpp::Time last_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<rm_competition_interfaces::msg::ChassisMode>::SharedPtr
    subscription_;
  rclcpp::Publisher<rm_competition_interfaces::msg::ChassisMode>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisModeGateNode>());
  rclcpp::shutdown();
  return 0;
}
