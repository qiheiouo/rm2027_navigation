#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class ChassisCommandDisturbance : public rclcpp::Node
{
public:
  ChassisCommandDisturbance()
  : Node("chassis_command_disturbance")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/simulation/chassis/cmd_vel");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/simulation/chassis/cmd_vel_applied");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    forward_scale_ = declare_parameter<double>("forward_scale", 1.0);
    lateral_positive_scale_ =
      declare_parameter<double>("lateral_positive_scale", 1.0);
    lateral_negative_scale_ =
      declare_parameter<double>("lateral_negative_scale", 1.0);
    angular_scale_ = declare_parameter<double>("angular_scale", 1.0);
    lateral_time_constant_sec_ =
      declare_parameter<double>("lateral_time_constant_sec", 0.0);
    angular_time_constant_sec_ =
      declare_parameter<double>("angular_time_constant_sec", 0.0);

    validateParameters();
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_,
      10,
      std::bind(
        &ChassisCommandDisturbance::handleCommand,
        this,
        std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ChassisCommandDisturbance::publishAppliedCommand, this));
    last_update_ = std::chrono::steady_clock::now();

    RCLCPP_WARN(
      get_logger(),
      "Simulation-only chassis disturbance: forward_scale=%.3f, "
      "lateral_scale=(+%.3f/-%.3f), angular_scale=%.3f, "
      "lateral_tau=%.3f s, angular_tau=%.3f s.",
      forward_scale_,
      lateral_positive_scale_,
      lateral_negative_scale_,
      angular_scale_,
      lateral_time_constant_sec_,
      angular_time_constant_sec_);
  }

private:
  void validateParameters() const
  {
    const bool invalid =
      !std::isfinite(publish_rate_hz_) ||
      !std::isfinite(forward_scale_) ||
      !std::isfinite(lateral_positive_scale_) ||
      !std::isfinite(lateral_negative_scale_) ||
      !std::isfinite(angular_scale_) ||
      !std::isfinite(lateral_time_constant_sec_) ||
      !std::isfinite(angular_time_constant_sec_) ||
      publish_rate_hz_ <= 0.0 ||
      forward_scale_ < 0.0 || forward_scale_ > 2.0 ||
      lateral_positive_scale_ < 0.0 || lateral_positive_scale_ > 2.0 ||
      lateral_negative_scale_ < 0.0 || lateral_negative_scale_ > 2.0 ||
      angular_scale_ < 0.0 || angular_scale_ > 2.0 ||
      lateral_time_constant_sec_ < 0.0 ||
      lateral_time_constant_sec_ > 2.0 ||
      angular_time_constant_sec_ < 0.0 ||
      angular_time_constant_sec_ > 2.0;
    if (invalid) {
      throw std::invalid_argument(
              "chassis disturbance scales must be within [0, 2], "
              "time constants within [0, 2] s, and publish rate positive");
    }
  }

  void handleCommand(const geometry_msgs::msg::Twist::SharedPtr input)
  {
    target_ = *input;
    target_.linear.x *= forward_scale_;
    target_.linear.y *=
      target_.linear.y >= 0.0 ?
      lateral_positive_scale_ : lateral_negative_scale_;
    target_.angular.z *= angular_scale_;
    target_.linear.z = 0.0;
    target_.angular.x = 0.0;
    target_.angular.y = 0.0;
  }

  static double firstOrderStep(
    double current,
    double target,
    double time_constant_sec,
    double delta_sec)
  {
    if (time_constant_sec <= 0.0) {
      return target;
    }
    const double alpha = std::clamp(
      delta_sec / (time_constant_sec + delta_sec), 0.0, 1.0);
    return current + alpha * (target - current);
  }

  void publishAppliedCommand()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double delta_sec =
      std::chrono::duration<double>(current_time - last_update_).count();
    last_update_ = current_time;

    applied_.linear.x = target_.linear.x;
    applied_.linear.y = firstOrderStep(
      applied_.linear.y,
      target_.linear.y,
      lateral_time_constant_sec_,
      delta_sec);
    applied_.angular.z = firstOrderStep(
      applied_.angular.z,
      target_.angular.z,
      angular_time_constant_sec_,
      delta_sec);
    publisher_->publish(applied_);
  }

  std::string input_topic_;
  std::string output_topic_;
  double publish_rate_hz_;
  double forward_scale_;
  double lateral_positive_scale_;
  double lateral_negative_scale_;
  double angular_scale_;
  double lateral_time_constant_sec_;
  double angular_time_constant_sec_;

  geometry_msgs::msg::Twist target_;
  geometry_msgs::msg::Twist applied_;
  std::chrono::steady_clock::time_point last_update_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisCommandDisturbance>());
  rclcpp::shutdown();
  return 0;
}
