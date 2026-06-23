#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

class MovingObstacleController : public rclcpp::Node
{
public:
  MovingObstacleController()
  : Node("moving_obstacle_controller")
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/simulation/moving_obstacle/target");
    center_ = declare_parameter<double>("center", 0.0);
    amplitude_ = declare_parameter<double>("amplitude", 0.9);
    period_ = declare_parameter<double>("period", 8.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    if (!std::isfinite(center_) || !std::isfinite(amplitude_) ||
      !std::isfinite(period_) || !std::isfinite(publish_rate_) ||
      amplitude_ < 0.0 || period_ <= 0.0 || publish_rate_ <= 0.0)
    {
      throw std::invalid_argument("Moving obstacle parameters must be finite and positive");
    }

    publisher_ = create_publisher<std_msgs::msg::Float64>(output_topic_, 1);
    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&MovingObstacleController::publishTarget, this));

    RCLCPP_INFO(
      get_logger(),
      "Simulation moving obstacle: topic=%s center=%.3f amplitude=%.3f period=%.3f. "
      "No TF, odometry, command velocity, or navigation goal is published.",
      output_topic_.c_str(), center_, amplitude_, period_);
  }

private:
  void publishTarget()
  {
    const double simulation_seconds = now().seconds();
    std_msgs::msg::Float64 message;
    message.data = center_ + amplitude_ * std::sin(kTwoPi * simulation_seconds / period_);
    publisher_->publish(message);
  }

  std::string output_topic_;
  double center_;
  double amplitude_;
  double period_;
  double publish_rate_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingObstacleController>());
  rclcpp::shutdown();
  return 0;
}
