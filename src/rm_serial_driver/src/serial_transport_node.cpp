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

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive and finite");
    }
    if (!std::isfinite(cmd_vel_timeout_sec_) || cmd_vel_timeout_sec_ < 0.0) {
      throw std::runtime_error("cmd_vel_timeout_sec must be finite and non-negative");
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

    RCLCPP_WARN(
      get_logger(),
      "REAL SERIAL TRANSPORT ENABLED: device=%s baudrate=%d profile=%s "
      "limits=(%.3f, %.3f, %.3f). This node writes bytes to the lower controller "
      "but publishes no TF, odometry, referee state, or navigation goals.",
      device_.c_str(), baudrate_, protocol_profile_text_.c_str(), max_vx_, max_vy_, max_wz_);
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
  std::uint8_t sequence_ = 0;
  rm_serial_driver::legacy_v1::Command latest_command_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  std::unique_ptr<PosixSerialPort> serial_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTransportNode>());
  rclcpp::shutdown();
  return 0;
}
