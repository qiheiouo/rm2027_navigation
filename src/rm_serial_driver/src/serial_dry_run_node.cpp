#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_serial_driver/protocol.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace
{

enum class ProtocolProfile
{
  LegacyNoCrc,
  HpmPayloadCrc,
};

ProtocolProfile parse_profile(const std::string & profile)
{
  if (profile == "legacy_v1_no_crc") {
    return ProtocolProfile::LegacyNoCrc;
  }
  if (profile == "hpm_crc_v1") {
    return ProtocolProfile::HpmPayloadCrc;
  }
  throw std::runtime_error(
    "Unsupported protocol_profile '" + profile +
    "'. Expected legacy_v1_no_crc or hpm_crc_v1.");
}

double finite_or_zero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

double clamp_symmetric(double value, double limit)
{
  const double safe_limit = std::max(0.0, finite_or_zero(limit));
  return std::clamp(finite_or_zero(value), -safe_limit, safe_limit);
}

template<std::size_t N>
std::vector<std::uint8_t> to_vector(const std::array<std::uint8_t, N> & frame)
{
  return std::vector<std::uint8_t>(frame.begin(), frame.end());
}

}  // namespace

class SerialDryRunNode : public rclcpp::Node
{
public:
  SerialDryRunNode()
  : Node("serial_dry_run_node")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    mock_tx_topic_ = declare_parameter<std::string>("mock_tx_topic", "/serial/mock_tx");
    protocol_profile_text_ =
      declare_parameter<std::string>("protocol_profile", "legacy_v1_no_crc");
    protocol_profile_ = parse_profile(protocol_profile_text_);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.2);
    max_vx_ = declare_parameter<double>("max_vx", 0.8);
    max_vy_ = declare_parameter<double>("max_vy", 0.5);
    max_wz_ = declare_parameter<double>("max_wz", 1.2);
    navigation_state_ =
      static_cast<std::uint8_t>(declare_parameter<int>("navigation_state", 0));
    remake_command_ =
      static_cast<std::uint8_t>(declare_parameter<int>("remake_command", 0));
    bullet_command_ =
      static_cast<std::uint8_t>(declare_parameter<int>("bullet_command", 0));
    allow_restart_ =
      static_cast<std::uint8_t>(declare_parameter<int>("allow_restart", 0));

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive and finite");
    }
    if (!std::isfinite(cmd_vel_timeout_sec_) || cmd_vel_timeout_sec_ < 0.0) {
      throw std::runtime_error("cmd_vel_timeout_sec must be finite and non-negative");
    }

    mock_tx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      mock_tx_topic_, rclcpp::SystemDefaultsQoS());
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&SerialDryRunNode::handleCmdVel, this, std::placeholders::_1));

    last_cmd_time_ = std::chrono::steady_clock::now() - std::chrono::hours(24);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&SerialDryRunNode::publishFrame, this));

    RCLCPP_WARN(
      get_logger(),
      "Serial dry-run started: cmd_vel=%s mock_tx=%s profile=%s. "
      "No serial device is opened and no TF/odom/nav goal is published.",
      cmd_vel_topic_.c_str(), mock_tx_topic_.c_str(),
      protocol_profile_text_.c_str());
  }

private:
  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.z) || !std::isfinite(msg->angular.x) ||
      !std::isfinite(msg->angular.y))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring non-finite unused Twist dimensions and encoding only vx/vy/wz.");
    } else if (std::abs(msg->linear.z) > 1.0e-6 || std::abs(msg->angular.x) > 1.0e-6 ||
      std::abs(msg->angular.y) > 1.0e-6)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Twist linear.z/angular.x/angular.y are ignored by the chassis protocol.");
    }

    latest_command_.vx = static_cast<float>(clamp_symmetric(msg->linear.x, max_vx_));
    latest_command_.vy = static_cast<float>(clamp_symmetric(msg->linear.y, max_vy_));
    latest_command_.wz = static_cast<float>(clamp_symmetric(msg->angular.z, max_wz_));
    latest_command_.navigation_state = navigation_state_;
    latest_command_.remake_command = remake_command_;
    latest_command_.bullet_command = bullet_command_;
    latest_command_.allow_restart = allow_restart_;
    last_cmd_time_ = std::chrono::steady_clock::now();
  }

  void publishFrame()
  {
    auto command = latest_command_;
    const auto now = std::chrono::steady_clock::now();
    const auto age = std::chrono::duration<double>(now - last_cmd_time_).count();
    if (age > cmd_vel_timeout_sec_) {
      command.vx = 0.0F;
      command.vy = 0.0F;
      command.wz = 0.0F;
    }
    command.sequence = sequence_++;

    std::vector<std::uint8_t> bytes;
    if (protocol_profile_ == ProtocolProfile::LegacyNoCrc) {
      const auto frame = rm_serial_driver::legacy_v1::encode_command(command);
      if (!frame.has_value()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to encode legacy frame");
        return;
      }
      bytes = to_vector(*frame);
    } else {
      const auto frame = rm_serial_driver::hpm_crc_v1::encode_command(command);
      if (!frame.has_value()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to encode HPM frame");
        return;
      }
      bytes = to_vector(*frame);
    }

    std_msgs::msg::UInt8MultiArray message;
    message.data = std::move(bytes);
    mock_tx_pub_->publish(message);
  }

  std::string cmd_vel_topic_;
  std::string mock_tx_topic_;
  std::string protocol_profile_text_;
  ProtocolProfile protocol_profile_;
  double publish_rate_hz_;
  double cmd_vel_timeout_sec_;
  double max_vx_;
  double max_vy_;
  double max_wz_;
  std::uint8_t navigation_state_;
  std::uint8_t remake_command_;
  std::uint8_t bullet_command_;
  std::uint8_t allow_restart_;
  std::uint8_t sequence_ = 0;
  rm_serial_driver::legacy_v1::Command latest_command_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr mock_tx_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialDryRunNode>());
  rclcpp::shutdown();
  return 0;
}
