#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/gimbal_state.hpp"
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
    if (stale_timeout_sec_ < 0.0) {
      RCLCPP_WARN(get_logger(), "stale_timeout_sec must not be negative. Falling back to 0.2 s.");
      stale_timeout_sec_ = 0.2;
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);

    if (use_input_) {
      gimbal_sub_ = create_subscription<rm_competition_interfaces::msg::GimbalState>(
        input_topic_, 10,
        [this](const rm_competition_interfaces::msg::GimbalState::SharedPtr msg) {
          handleInput(*msg);
        });
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publishJointState();
      });

    if (use_input_) {
      RCLCPP_INFO(
        get_logger(),
        "Real gimbal input mode: %s -> %s[%s]. Invalid or stale input pauses output.",
        input_topic_.c_str(), output_topic_.c_str(), joint_name_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Placeholder gimbal mode: publishing fixed %.3f rad on %s[%s].",
        placeholder_yaw_rad_, output_topic_.c_str(), joint_name_.c_str());
    }
  }

private:
  void handleInput(const rm_competition_interfaces::msg::GimbalState & msg)
  {
    if (!msg.valid || !msg.online) {
      have_valid_input_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input %s reports invalid or offline gimbal state.", input_topic_.c_str());
      return;
    }
    if (!std::isfinite(msg.relative_yaw_rad) || !std::isfinite(msg.yaw_rate_rad_s)) {
      have_valid_input_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input %s has non-finite gimbal state.", input_topic_.c_str());
      return;
    }
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      have_valid_input_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input %s has a zero sample timestamp.", input_topic_.c_str());
      return;
    }
    const rclcpp::Time sample_stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double sample_age = (now() - sample_stamp).seconds();
    if (sample_age < -stale_timeout_sec_ || sample_age > stale_timeout_sec_) {
      have_valid_input_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input %s sample age %.3f s exceeds the freshness window.",
        input_topic_.c_str(), sample_age);
      return;
    }

    latest_yaw_rad_ = msg.relative_yaw_rad;
    latest_velocity_rad_s_ = msg.yaw_rate_rad_s;
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
        have_valid_input_ = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Gimbal input is stale for %.3f s. Joint-state publication is paused.", age);
      }
    }

    if (use_input_ && !using_input) {
      return;
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

  rclcpp::Subscription<rm_competition_interfaces::msg::GimbalState>::SharedPtr gimbal_sub_;
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
