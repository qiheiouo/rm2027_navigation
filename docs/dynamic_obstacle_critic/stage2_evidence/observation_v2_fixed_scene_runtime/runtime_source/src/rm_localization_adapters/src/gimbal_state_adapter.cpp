#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class GimbalStateAdapter : public rclcpp::Node
{
public:
  GimbalStateAdapter()
  : Node("gimbal_state_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/gimbal/state");
    output_topic_ = declare_parameter<std::string>("output_topic", "/joint_states");
    joint_name_ = declare_parameter<std::string>("joint_name", "gimbal_yaw_joint");
    use_input_ = declare_parameter<bool>("use_input", false);
    placeholder_yaw_rad_ = declare_parameter<double>("placeholder_yaw_rad", 0.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.2);

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive. Falling back to 50 Hz.");
      publish_rate_hz_ = 50.0;
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);

    if (use_input_) {
      joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          handleInput(*msg);
        });
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publishJointState();
      });

    RCLCPP_WARN(
      get_logger(),
      "Phase 1 skeleton: publishing %s on %s. Real hardware must provide "
      "timestamped gimbal yaw before rotating-gimbal localization is accepted.",
      joint_name_.c_str(), output_topic_.c_str());
  }

private:
  void handleInput(const sensor_msgs::msg::JointState & msg)
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name_);
    if (it == msg.name.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input %s does not contain joint '%s'.",
        input_topic_.c_str(), joint_name_.c_str());
      return;
    }

    const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
    if (index >= msg.position.size() || !std::isfinite(msg.position[index])) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input joint '%s' has no finite position.", joint_name_.c_str());
      return;
    }

    latest_yaw_rad_ = msg.position[index];
    latest_velocity_rad_s_ =
      index < msg.velocity.size() && std::isfinite(msg.velocity[index]) ? msg.velocity[index] : 0.0;
    latest_input_stamp_msg_ = msg.header.stamp;
    latest_input_received_time_ = now();
    have_valid_input_ = true;
  }

  void publishJointState()
  {
    double yaw_rad = placeholder_yaw_rad_;
    double yaw_velocity_rad_s = 0.0;
    bool using_input = false;

    if (use_input_ && have_valid_input_) {
      const double age = (now() - latest_input_received_time_).seconds();
      if (age <= stale_timeout_sec_) {
        yaw_rad = latest_yaw_rad_;
        yaw_velocity_rad_s = latest_velocity_rad_s_;
        using_input = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Gimbal input is stale for %.3f s. Publishing placeholder yaw.", age);
      }
    }

    sensor_msgs::msg::JointState joint_state;
    if (using_input && (latest_input_stamp_msg_.sec != 0 || latest_input_stamp_msg_.nanosec != 0)) {
      joint_state.header.stamp = latest_input_stamp_msg_;
    } else {
      joint_state.header.stamp = now();
    }
    joint_state.name = {joint_name_};
    joint_state.position = {yaw_rad};
    joint_state.velocity = {yaw_velocity_rad_s};
    joint_pub_->publish(joint_state);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string joint_name_;
  bool use_input_;
  double placeholder_yaw_rad_;
  double publish_rate_hz_;
  double stale_timeout_sec_;

  double latest_yaw_rad_ = 0.0;
  double latest_velocity_rad_s_ = 0.0;
  builtin_interfaces::msg::Time latest_input_stamp_msg_;
  rclcpp::Time latest_input_received_time_;
  bool have_valid_input_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalStateAdapter>());
  rclcpp::shutdown();
  return 0;
}
