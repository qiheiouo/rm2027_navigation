#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/operator_navigation_target.hpp"
#include "rm_competition_interfaces/msg/referee_state.hpp"
#include "rm_serial_driver/protocol.hpp"

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

std::uint8_t parse_operator_goal_coordinate_system(const std::string & value)
{
  using Message = rm_competition_interfaces::msg::OperatorNavigationTarget;
  if (value == "unknown") {
    return Message::COORDINATE_SYSTEM_UNKNOWN;
  }
  if (value == "referee_field") {
    return Message::COORDINATE_SYSTEM_REFEREE_FIELD;
  }
  if (value == "map") {
    return Message::COORDINATE_SYSTEM_MAP;
  }
  throw std::runtime_error(
    "Unsupported operator_goal_coordinate_system '" + value +
    "'. Expected unknown, referee_field, or map.");
}

speed_t parse_baudrate(const int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::runtime_error("Unsupported baudrate: " + std::to_string(baudrate));
  }
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

class PosixSerialPort
{
public:
  PosixSerialPort(const std::string & device, int baudrate)
  {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "open " + device);
    }

    termios tty {};
    if (::tcgetattr(fd_, &tty) != 0) {
      close();
      throw std::system_error(errno, std::generic_category(), "tcgetattr " + device);
    }

    ::cfmakeraw(&tty);
    const speed_t speed = parse_baudrate(baudrate);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    tty.c_cflag |= static_cast<unsigned int>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<unsigned int>(~CSTOPB);
    tty.c_cflag &= static_cast<unsigned int>(~CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close();
      throw std::system_error(errno, std::generic_category(), "tcsetattr " + device);
    }
  }

  ~PosixSerialPort()
  {
    close();
  }

  PosixSerialPort(const PosixSerialPort &) = delete;
  PosixSerialPort & operator=(const PosixSerialPort &) = delete;

  bool write_all(const std::vector<std::uint8_t> & bytes, std::string & error)
  {
    std::size_t written = 0;
    while (written < bytes.size()) {
      const ssize_t ret = ::write(fd_, bytes.data() + written, bytes.size() - written);
      if (ret > 0) {
        written += static_cast<std::size_t>(ret);
        continue;
      }
      if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        continue;
      }
      error = std::strerror(errno);
      return false;
    }
    return true;
  }

  ssize_t read_some(std::uint8_t * bytes, std::size_t capacity, std::string & error)
  {
    const ssize_t ret = ::read(fd_, bytes, capacity);
    if (ret >= 0) {
      return ret;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    error = std::strerror(errno);
    return -1;
  }

private:
  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_ = -1;
};

}  // namespace

class SerialTransportNode : public rclcpp::Node
{
public:
  SerialTransportNode()
  : Node("serial_transport_node")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    protocol_profile_text_ =
      declare_parameter<std::string>("protocol_profile", "legacy_v1_no_crc");
    protocol_profile_ = parse_profile(protocol_profile_text_);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.2);
    max_vx_ = declare_parameter<double>("max_vx", 0.15);
    max_vy_ = declare_parameter<double>("max_vy", 0.15);
    max_wz_ = declare_parameter<double>("max_wz", 0.30);
    navigation_state_ =
      static_cast<std::uint8_t>(declare_parameter<int>("navigation_state", 0));
    remake_command_ =
      static_cast<std::uint8_t>(declare_parameter<int>("remake_command", 0));
    bullet_command_ =
      static_cast<std::uint8_t>(declare_parameter<int>("bullet_command", 0));
    allow_restart_ =
      static_cast<std::uint8_t>(declare_parameter<int>("allow_restart", 0));
    referee_rx_enabled_ = declare_parameter<bool>("referee_rx_enabled", false);
    referee_raw_topic_ =
      declare_parameter<std::string>("referee_raw_topic", "/referee/state_raw");
    operator_goal_rx_enabled_ =
      declare_parameter<bool>("operator_goal_rx_enabled", false);
    operator_goal_raw_topic_ = declare_parameter<std::string>(
      "operator_goal_raw_topic", "/operator/navigation_target_raw");
    const auto operator_goal_coordinate_system_text = declare_parameter<std::string>(
      "operator_goal_coordinate_system", "unknown");
    operator_goal_coordinate_system_ = parse_operator_goal_coordinate_system(
      operator_goal_coordinate_system_text);
    read_poll_rate_hz_ = declare_parameter<double>("read_poll_rate_hz", 200.0);
    blue_team_robot_id_min_ =
      declare_parameter<int>("blue_team_robot_id_min", 100);

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive and finite");
    }
    if (!std::isfinite(cmd_vel_timeout_sec_) || cmd_vel_timeout_sec_ < 0.0) {
      throw std::runtime_error("cmd_vel_timeout_sec must be finite and non-negative");
    }
    if (!std::isfinite(read_poll_rate_hz_) || read_poll_rate_hz_ <= 0.0) {
      throw std::runtime_error("read_poll_rate_hz must be positive and finite");
    }
    if (blue_team_robot_id_min_ <= 0 || blue_team_robot_id_min_ > 255) {
      throw std::runtime_error("blue_team_robot_id_min must be in [1, 255]");
    }
    if ((referee_rx_enabled_ || operator_goal_rx_enabled_) &&
      protocol_profile_ != ProtocolProfile::HpmPayloadCrc)
    {
      throw std::runtime_error(
              "serial feedback receive requires protocol_profile:=hpm_crc_v1 because the "
              "currently flashed lower controller only replies to CRC-valid commands");
    }

    serial_ = std::make_unique<PosixSerialPort>(device_, baudrate_);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&SerialTransportNode::handleCmdVel, this, std::placeholders::_1));

    last_cmd_time_ = std::chrono::steady_clock::now() - std::chrono::hours(24);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&SerialTransportNode::writeFrame, this));

    if (referee_rx_enabled_) {
      referee_pub_ = create_publisher<rm_competition_interfaces::msg::RefereeState>(
        referee_raw_topic_, rclcpp::QoS(10));
    }
    if (operator_goal_rx_enabled_) {
      operator_goal_pub_ =
        create_publisher<rm_competition_interfaces::msg::OperatorNavigationTarget>(
          operator_goal_raw_topic_, rclcpp::QoS(10));
    }
    if (referee_rx_enabled_ || operator_goal_rx_enabled_) {
      read_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / read_poll_rate_hz_)),
        std::bind(&SerialTransportNode::readFrames, this));
    }

    RCLCPP_WARN(
      get_logger(),
      "REAL SERIAL TRANSPORT ENABLED: device=%s baudrate=%d profile=%s "
      "limits=(%.3f, %.3f, %.3f) referee_rx=%s operator_goal_rx=%s. "
      "This node owns serial bytes only "
      "and publishes no TF, odometry, or navigation goals.",
      device_.c_str(), baudrate_, protocol_profile_text_.c_str(), max_vx_, max_vy_, max_wz_,
      referee_rx_enabled_ ? "true" : "false",
      operator_goal_rx_enabled_ ? "true" : "false");
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

  void writeFrame()
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

    std::string error;
    if (!serial_->write_all(bytes, error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Failed to write serial frame to %s: %s", device_.c_str(), error.c_str());
    }
  }

  void readFrames()
  {
    std::array<std::uint8_t, 512> bytes{};
    for (int attempt = 0; attempt < 8; ++attempt) {
      std::string error;
      const ssize_t count = serial_->read_some(bytes.data(), bytes.size(), error);
      if (count < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Failed to read serial feedback from %s: %s", device_.c_str(), error.c_str());
        return;
      }
      if (count == 0) {
        break;
      }
      feedback_stream_.append(bytes.data(), static_cast<std::size_t>(count));
    }

    rm_serial_driver::hpm_crc_v1::Frame frame;
    while (feedback_stream_.pop(frame)) {
      const auto feedback = rm_serial_driver::hpm_referee_v1::decode_feedback(frame);
      if (!feedback.has_value()) {
        RCLCPP_DEBUG(
          get_logger(), "Ignoring CRC-valid lower-controller payload of length %zu",
          frame.payload.size());
        continue;
      }
      if (referee_rx_enabled_) {
        publishRefereeState(*feedback);
      }
      if (operator_goal_rx_enabled_) {
        publishOperatorNavigationTarget(*feedback);
      }
    }
  }

  void publishRefereeState(const rm_serial_driver::hpm_referee_v1::Feedback & feedback)
  {
    rm_competition_interfaces::msg::RefereeState message;
    message.header.stamp = get_clock()->now();
    message.valid = feedback.robot_id != 0U;
    message.game_progress = feedback.game_progress;
    message.stage_remain_time = feedback.stage_remain_time;
    message.robot_id = feedback.robot_id;
    message.current_hp = feedback.current_hp;
    const bool blue_team =
      static_cast<int>(feedback.robot_id) >= blue_team_robot_id_min_;
    message.self_outpost_hp =
      blue_team ? feedback.blue_outpost_hp : feedback.red_outpost_hp;
    message.enemy_outpost_hp =
      blue_team ? feedback.red_outpost_hp : feedback.blue_outpost_hp;
    message.projectile_allowance_17mm = feedback.projectile_allowance_17mm;
    message.remaining_gold_coin = feedback.remaining_gold_coin;
    referee_pub_->publish(message);
  }

  void publishOperatorNavigationTarget(
    const rm_serial_driver::hpm_referee_v1::Feedback & feedback)
  {
    rm_competition_interfaces::msg::OperatorNavigationTarget message;
    message.header.stamp = get_clock()->now();
    message.target_x = feedback.target_position_x;
    message.target_y = feedback.target_position_y;
    message.robot_id = feedback.robot_id;
    message.coordinate_system = operator_goal_coordinate_system_;
    message.transport_valid =
      feedback.robot_id != 0U && std::isfinite(feedback.target_position_x) &&
      std::isfinite(feedback.target_position_y);
    operator_goal_pub_->publish(message);
  }

  std::string cmd_vel_topic_;
  std::string device_;
  int baudrate_;
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
  bool referee_rx_enabled_;
  std::string referee_raw_topic_;
  bool operator_goal_rx_enabled_;
  std::string operator_goal_raw_topic_;
  std::uint8_t operator_goal_coordinate_system_;
  double read_poll_rate_hz_;
  int blue_team_robot_id_min_;
  std::uint8_t sequence_ = 0;
  rm_serial_driver::legacy_v1::Command latest_command_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  std::unique_ptr<PosixSerialPort> serial_;
  rm_serial_driver::hpm_crc_v1::StreamDecoder feedback_stream_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::RefereeState>::SharedPtr referee_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::OperatorNavigationTarget>::SharedPtr
    operator_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTransportNode>());
  rclcpp::shutdown();
  return 0;
}
