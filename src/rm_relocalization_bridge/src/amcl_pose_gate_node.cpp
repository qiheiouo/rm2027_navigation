#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rm_competition_interfaces/msg/localization_status.hpp"

#include "rm_relocalization_bridge/global_pose_gate.hpp"

class GlobalPoseGateNode : public rclcpp::Node
{
public:
  GlobalPoseGateNode()
  : Node("global_pose_gate")
  {
    started_at_ = now();
    recovery_after_ns_ = started_at_.nanoseconds();
    last_clock_ns_ = recovery_after_ns_;
    producer_id_ = std::to_string(std::random_device{}()) + ":" +
      std::to_string(std::random_device{}());
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
      !std::isfinite(validity_timeout_sec_) ||
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
    status_pub_ = create_publisher<rm_competition_interfaces::msg::LocalizationStatus>(
      valid_topic_ + "/status", rclcpp::QoS(1).reliable());
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
    checkTimeout(false);
    std::string reason;
    const int64_t stamp_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
      message.header.stamp.nanosec;
    if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
      !rm_relocalization_bridge::validateGlobalPose(
        message, now().nanoseconds(), limits_, &reason) ||
      stamp_ns <= recovery_after_ns_ || stamp_ns <= last_pose_ns_)
    {
      invalidate();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting backend pose: %s",
        reason.empty() ? "non-monotonic or pre-session timestamp" : reason.c_str());
      return;
    }

    output_pub_->publish(message);
    accepted_pose_ = message;
    last_pose_ns_ = stamp_ns;
    // The existing 100 ms timeout timer supplies the source-deadline upper bound.
    valid_until_ = now() + rclcpp::Duration::from_seconds(validity_timeout_sec_ + 0.1);
    last_accepted_ = std::chrono::steady_clock::now();
    has_accepted_pose_ = true;
    publishValid(true);
    publishStatus();
  }

  void invalidate()
  {
    has_accepted_pose_ = false;
    recovery_after_ns_ = std::max(recovery_after_ns_, last_pose_ns_);
    publishValid(false);
    publishStatus();
  }

  void checkTimeout(bool publish = true)
  {
    const auto clock_ns = now().nanoseconds();
    if (started_at_.nanoseconds() <= 0 && clock_ns > 0) {
      started_at_ = now();
      recovery_after_ns_ = clock_ns;
    }
    const bool clock_reversed = clock_ns < last_clock_ns_;
    last_clock_ns_ = std::max(last_clock_ns_, clock_ns);
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_accepted_).count();
    if (clock_reversed || (publish && has_accepted_pose_ &&
      (age > validity_timeout_sec_ || clock_ns > valid_until_.nanoseconds())))
    {
      if (clock_reversed) {recovery_after_ns_ = last_clock_ns_;}
      invalidate();
    }
    if (publish) {publishStatus();}
  }

  void publishStatus()
  {
    rm_competition_interfaces::msg::LocalizationStatus message;
    message.header.stamp = now();
    message.header.frame_id = limits_.map_frame;
    message.producer_id = producer_id_;
    message.started_at = started_at_;
    message.sequence = ++sequence_;
    message.valid = valid_state_ && has_accepted_pose_;
    message.valid_until = valid_until_;
    message.pose = accepted_pose_;
    status_pub_->publish(message);
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

  std::string producer_id_;
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time valid_until_{0, 0, RCL_ROS_TIME};
  int64_t recovery_after_ns_{0}, last_pose_ns_{0}, last_clock_ns_{0};
  uint64_t sequence_{0};
  geometry_msgs::msg::PoseWithCovarianceStamped accepted_pose_;
  rclcpp::Publisher<rm_competition_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
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
