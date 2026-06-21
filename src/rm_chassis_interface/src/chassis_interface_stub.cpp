#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class ChassisInterfaceStub : public rclcpp::Node
{
public:
  ChassisInterfaceStub()
  : Node("chassis_interface_stub")
  {
    max_vx_ = declare_parameter<double>("limits.max_vx", 1.0);
    max_vy_ = declare_parameter<double>("limits.max_vy", 1.0);
    max_wz_ = declare_parameter<double>("limits.max_wz", 2.0);
    watchdog_timeout_sec_ = declare_parameter<double>("watchdog_timeout_sec", 0.5);
    publish_twist_raw_ = declare_parameter<bool>("publish_twist_raw", true);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    mock_output_cmd_vel_topic_ =
      declare_parameter<std::string>("mock_output_cmd_vel_topic", "");

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&ChassisInterfaceStub::handleCmdVel, this, std::placeholders::_1));

    if (!mock_output_cmd_vel_topic_.empty()) {
      if (mock_output_cmd_vel_topic_ == cmd_vel_topic_) {
        RCLCPP_ERROR(
          get_logger(),
          "mock_output_cmd_vel_topic must differ from cmd_vel_topic; relay is disabled.");
      } else {
        mock_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
          mock_output_cmd_vel_topic_, 10);
      }
    }

    if (publish_twist_raw_) {
      twist_raw_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/chassis/twist_raw", 10);
    }

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ChassisInterfaceStub::checkWatchdog, this));

    RCLCPP_WARN(
      get_logger(),
      "Phase 1 chassis stub only: no TF, no odom, no nav goals, no real serial.");
  }

private:
  static double clamp(double value, double limit)
  {
    return std::clamp(value, -std::abs(limit), std::abs(limit));
  }

  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) ||
      !std::isfinite(msg->linear.y) ||
      !std::isfinite(msg->angular.z))
    {
      RCLCPP_ERROR(get_logger(), "Reject non-finite /cmd_vel command.");
      return;
    }

    if (std::abs(msg->linear.z) > 1e-6 ||
      std::abs(msg->angular.x) > 1e-6 ||
      std::abs(msg->angular.y) > 1e-6)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported /cmd_vel dimensions are non-zero and will be ignored.");
    }

    last_cmd_.linear.x = clamp(msg->linear.x, max_vx_);
    last_cmd_.linear.y = clamp(msg->linear.y, max_vy_);
    last_cmd_.angular.z = clamp(msg->angular.z, max_wz_);
    last_cmd_time_ = now();
    have_cmd_ = true;
    watchdog_active_ = false;

    publishMockCommand(last_cmd_);
    publishMockFeedback(last_cmd_);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Mock chassis packet vx=%.3f vy=%.3f wz=%.3f",
      last_cmd_.linear.x, last_cmd_.linear.y, last_cmd_.angular.z);
  }

  void checkWatchdog()
  {
    if (!have_cmd_) {
      return;
    }

    const double age = (now() - last_cmd_time_).seconds();
    if (age > watchdog_timeout_sec_ && !watchdog_active_) {
      geometry_msgs::msg::Twist zero;
      last_cmd_ = zero;
      watchdog_active_ = true;
      publishMockCommand(zero);
      publishMockFeedback(zero);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Watchdog timeout. Mock chassis command forced to zero.");
    }
  }

  void publishMockCommand(const geometry_msgs::msg::Twist & twist)
  {
    if (mock_cmd_pub_) {
      mock_cmd_pub_->publish(twist);
    }
  }

  void publishMockFeedback(const geometry_msgs::msg::Twist & twist)
  {
    if (!publish_twist_raw_ || !twist_raw_pub_) {
      return;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped feedback;
    feedback.header.stamp = now();
    feedback.header.frame_id = "base_link";
    feedback.twist.twist = twist;
    feedback.twist.covariance[0] = 999.0;
    feedback.twist.covariance[7] = 999.0;
    feedback.twist.covariance[35] = 999.0;
    twist_raw_pub_->publish(feedback);
  }

  double max_vx_;
  double max_vy_;
  double max_wz_;
  double watchdog_timeout_sec_;
  bool publish_twist_raw_;
  bool have_cmd_ = false;
  bool watchdog_active_ = false;
  std::string cmd_vel_topic_;
  std::string mock_output_cmd_vel_topic_;
  rclcpp::Time last_cmd_time_;
  geometry_msgs::msg::Twist last_cmd_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mock_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_raw_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisInterfaceStub>());
  rclcpp::shutdown();
  return 0;
}
