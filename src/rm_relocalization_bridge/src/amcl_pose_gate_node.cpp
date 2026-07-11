#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rm_relocalization_bridge/global_pose_gate.hpp"

class GlobalPoseGateNode : public rclcpp::Node
{
public:
  GlobalPoseGateNode()
  : Node("global_pose_gate")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/localization/amcl_pose_raw");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/localization/global_pose");
    valid_topic_ = declare_parameter<std::string>(
      "valid_topic", "/localization/amcl_backend_valid");
    limits_.map_frame = declare_parameter<std::string>("map_frame", "map");
    limits_.max_pose_age_sec = declare_parameter<double>("max_pose_age_sec", 0.5);
    limits_.max_future_sec = declare_parameter<double>("max_future_sec", 0.1);
    limits_.max_xy_variance = declare_parameter<double>("max_xy_variance", 1.0);
    limits_.max_yaw_variance = declare_parameter<double>("max_yaw_variance", 0.5);
    limits_.max_abs_z = declare_parameter<double>("max_abs_z", 0.25);
    limits_.max_abs_roll_pitch = declare_parameter<double>("max_abs_roll_pitch", 0.15);
    validity_timeout_sec_ = declare_parameter<double>("validity_timeout_sec", 1.0);

    if (
      limits_.max_pose_age_sec < 0.0 || limits_.max_future_sec < 0.0 ||
      limits_.max_xy_variance < 0.0 || limits_.max_yaw_variance < 0.0 ||
      limits_.max_abs_z < 0.0 || limits_.max_abs_roll_pitch < 0.0 ||
      validity_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("AMCL pose gate limits must be non-negative");
    }

    output_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, rclcpp::QoS(10).reliable());
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      valid_topic_, rclcpp::QoS(1).reliable().transient_local());
    input_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      input_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        handlePose(*message);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {checkTimeout();});

    publishValid(false);
    RCLCPP_INFO(
      get_logger(),
      "Global pose gate ready: %s -> %s. It publishes no TF.",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void handlePose(const geometry_msgs::msg::PoseWithCovarianceStamped & message)
  {
    std::string reason;
    if (!rm_relocalization_bridge::validateGlobalPose(
        message, now().nanoseconds(), limits_, &reason))
    {
      publishValid(false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting backend pose: %s", reason.c_str());
      return;
    }

    output_pub_->publish(message);
    last_accepted_ = std::chrono::steady_clock::now();
    has_accepted_pose_ = true;
    publishValid(true);
  }

  void checkTimeout()
  {
    if (!has_accepted_pose_) {
      return;
    }
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_accepted_).count();
    if (age > validity_timeout_sec_) {
      has_accepted_pose_ = false;
      publishValid(false);
      RCLCPP_WARN(get_logger(), "Global pose stream timed out after %.3f s", age);
    }
  }

  void publishValid(const bool value)
  {
    if (valid_state_initialized_ && valid_state_ == value) {
      return;
    }
    valid_state_initialized_ = true;
    valid_state_ = value;
    std_msgs::msg::Bool message;
    message.data = value;
    valid_pub_->publish(message);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string valid_topic_;
  rm_relocalization_bridge::GlobalPoseGateLimits limits_;
  double validity_timeout_sec_{1.0};
  bool has_accepted_pose_{false};
  bool valid_state_initialized_{false};
  bool valid_state_{false};
  std::chrono::steady_clock::time_point last_accepted_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr input_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPoseGateNode>());
  rclcpp::shutdown();
  return 0;
}
