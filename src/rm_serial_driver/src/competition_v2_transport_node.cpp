#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/gimbal_state.hpp"
#include "rm_competition_interfaces/msg/operator_navigation_target.hpp"
#include "rm_competition_interfaces/msg/posture_request.hpp"
#include "rm_competition_interfaces/msg/posture_state.hpp"
#include "rm_competition_interfaces/msg/referee_state.hpp"
#include "rm_competition_interfaces/msg/robot_posture.hpp"
#include "rm_competition_interfaces/msg/serial_connection_state.hpp"
#include "rm_serial_driver/competition_v2_protocol.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace v2 = rm_serial_driver::competition_v2;

namespace
{

speed_t parseBaudrate(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::runtime_error("Unsupported baudrate: " + std::to_string(baudrate));
  }
}

double finiteOrZero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

double clampSymmetric(double value, double limit)
{
  const double safe_limit = std::max(0.0, finiteOrZero(limit));
  return std::clamp(finiteOrZero(value), -safe_limit, safe_limit);
}

std::uint32_t makeBootId()
{
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto mixed = static_cast<std::uint64_t>(ticks) ^
    (static_cast<std::uint64_t>(::getpid()) << 32U);
  const auto value = static_cast<std::uint32_t>(mixed ^ (mixed >> 32U));
  return value == 0U ? 1U : value;
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
    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      closePort();
      throw std::system_error(errno, std::generic_category(), "tcgetattr " + device);
    }
    ::cfmakeraw(&tty);
    const auto speed = parseBaudrate(baudrate);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    tty.c_cflag |= static_cast<unsigned int>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<unsigned int>(~CSTOPB);
    tty.c_cflag &= static_cast<unsigned int>(~CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      closePort();
      throw std::system_error(errno, std::generic_category(), "tcsetattr " + device);
    }
  }

  ~PosixSerialPort() {closePort();}
  PosixSerialPort(const PosixSerialPort &) = delete;
  PosixSerialPort & operator=(const PosixSerialPort &) = delete;

  bool writeAll(const std::vector<std::uint8_t> & bytes, std::string & error)
  {
    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    while (written < bytes.size()) {
      const auto result = ::write(fd_, bytes.data() + written, bytes.size() - written);
      if (result > 0) {
        written += static_cast<std::size_t>(result);
      } else if (result < 0 && errno == EINTR) {
        continue;
      } else if (result == 0 ||
        (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)))
      {
        if (std::chrono::steady_clock::now() >= deadline) {
          error = "write timeout";
          return false;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      } else {
        error = std::strerror(errno);
        return false;
      }
    }
    return true;
  }

  ssize_t readSome(std::uint8_t * data, std::size_t capacity, std::string & error)
  {
    const auto result = ::read(fd_, data, capacity);
    if (result >= 0) {
      return result;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    error = std::strerror(errno);
    return -1;
  }

private:
  void closePort()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }
  int fd_ = -1;
};

}  // namespace

class CompetitionV2TransportNode : public rclcpp::Node
{
public:
  CompetitionV2TransportNode()
  : Node("competition_v2_transport_node"), local_boot_id_(makeBootId())
  {
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    posture_request_topic_ = declare_parameter<std::string>(
      "posture_request_topic", "/robot/posture/request");
    posture_state_topic_ = declare_parameter<std::string>(
      "posture_state_topic", "/robot/posture/state");
    gimbal_state_topic_ = declare_parameter<std::string>(
      "gimbal_state_topic", "/gimbal/state");
    referee_raw_topic_ = declare_parameter<std::string>(
      "referee_raw_topic", "/referee/state_raw");
    operator_goal_raw_topic_ = declare_parameter<std::string>(
      "operator_goal_raw_topic", "/operator/navigation_target_raw");
    connection_state_topic_ = declare_parameter<std::string>(
      "connection_state_topic", "/serial/connection_state");
    mock_tx_topic_ = declare_parameter<std::string>("mock_tx_topic", "/serial/v2/mock_tx");
    mock_rx_topic_ = declare_parameter<std::string>("mock_rx_topic", "/serial/v2/mock_rx");
    chassis_rate_hz_ = declare_parameter<double>("chassis_rate_hz", 100.0);
    posture_rate_hz_ = declare_parameter<double>("posture_rate_hz", 10.0);
    heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 2.0);
    read_poll_rate_hz_ = declare_parameter<double>("read_poll_rate_hz", 200.0);
    health_rate_hz_ = declare_parameter<double>("health_rate_hz", 10.0);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.2);
    connection_timeout_sec_ = declare_parameter<double>("connection_timeout_sec", 0.75);
    gimbal_timeout_sec_ = declare_parameter<double>("gimbal_timeout_sec", 0.2);
    posture_timeout_sec_ = declare_parameter<double>("posture_timeout_sec", 0.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.8);
    max_vy_ = declare_parameter<double>("max_vy", 0.5);
    max_wz_ = declare_parameter<double>("max_wz", 6.0);
    const auto required_capabilities = declare_parameter<std::int64_t>(
      "required_remote_capabilities", v2::CapabilityChassisCommand);
    if (required_capabilities < 0 ||
      required_capabilities > static_cast<std::int64_t>(
        std::numeric_limits<std::uint32_t>::max()))
    {
      throw std::runtime_error("required_remote_capabilities must fit uint32");
    }
    required_remote_capabilities_ = static_cast<std::uint32_t>(required_capabilities);

    validateParameters();
    if (!dry_run_) {
      serial_ = std::make_unique<PosixSerialPort>(device_, baudrate_);
    }

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    posture_state_pub_ = create_publisher<rm_competition_interfaces::msg::PostureState>(
      posture_state_topic_, latched);
    gimbal_state_pub_ = create_publisher<rm_competition_interfaces::msg::GimbalState>(
      gimbal_state_topic_, rclcpp::QoS(10));
    referee_pub_ = create_publisher<rm_competition_interfaces::msg::RefereeState>(
      referee_raw_topic_, rclcpp::QoS(10));
    operator_goal_pub_ = create_publisher<
      rm_competition_interfaces::msg::OperatorNavigationTarget>(
      operator_goal_raw_topic_, rclcpp::QoS(10));
    connection_pub_ = create_publisher<
      rm_competition_interfaces::msg::SerialConnectionState>(connection_state_topic_, latched);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&CompetitionV2TransportNode::handleCmdVel, this, std::placeholders::_1));
    posture_request_sub_ = create_subscription<rm_competition_interfaces::msg::PostureRequest>(
      posture_request_topic_, rclcpp::QoS(10),
      std::bind(&CompetitionV2TransportNode::handlePostureRequest, this, std::placeholders::_1));

    if (dry_run_) {
      mock_tx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
        mock_tx_topic_, rclcpp::QoS(100));
      mock_rx_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        mock_rx_topic_, rclcpp::QoS(100),
        [this](const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
          decoder_.append(message->data);
          processFrames();
        });
    }

    last_cmd_vel_time_ = steadyNow() - std::chrono::hours(24);
    last_heartbeat_time_ = steadyNow() - std::chrono::hours(24);
    last_gimbal_time_ = steadyNow() - std::chrono::hours(24);
    last_posture_time_ = steadyNow() - std::chrono::hours(24);
    start_time_ = steadyNow();

    chassis_timer_ = makeTimer(chassis_rate_hz_, [this]() {sendChassisCommand();});
    posture_timer_ = makeTimer(posture_rate_hz_, [this]() {resendPostureRequest();});
    heartbeat_timer_ = makeTimer(heartbeat_rate_hz_, [this]() {sendHeartbeat();});
    health_timer_ = makeTimer(health_rate_hz_, [this]() {publishHealth();});
    if (!dry_run_) {
      read_timer_ = makeTimer(read_poll_rate_hz_, [this]() {readSerial();});
    }

    publishHealth();
    RCLCPP_WARN(
      get_logger(),
      "competition_v2 transport started: dry_run=%s device=%s baud=%d boot_id=%u. "
      "It publishes no TF, odometry or navigation goals.",
      dry_run_ ? "true" : "false", device_.c_str(), baudrate_, local_boot_id_);
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static SteadyTime steadyNow() {return std::chrono::steady_clock::now();}

  rclcpp::TimerBase::SharedPtr makeTimer(double rate_hz, std::function<void()> callback)
  {
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    return create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::move(callback));
  }

  void validateParameters() const
  {
    const std::array<double, 10> positive_values{
      chassis_rate_hz_, posture_rate_hz_, heartbeat_rate_hz_, read_poll_rate_hz_,
      health_rate_hz_, connection_timeout_sec_, gimbal_timeout_sec_, posture_timeout_sec_,
      max_vx_, max_vy_};
    for (const auto value : positive_values) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error("competition_v2 rates, timeouts and velocity limits must be positive");
      }
    }
    if (!std::isfinite(cmd_vel_timeout_sec_) || cmd_vel_timeout_sec_ < 0.0 ||
      !std::isfinite(max_wz_) || max_wz_ <= 0.0)
    {
      throw std::runtime_error("cmd_vel_timeout_sec/max_wz parameter is invalid");
    }
  }

  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_chassis_.vx_mps = static_cast<float>(clampSymmetric(message->linear.x, max_vx_));
    latest_chassis_.vy_mps = static_cast<float>(clampSymmetric(message->linear.y, max_vy_));
    latest_chassis_.wz_rad_s = static_cast<float>(clampSymmetric(message->angular.z, max_wz_));
    latest_chassis_.enabled = true;
    last_cmd_vel_time_ = steadyNow();
  }

  void handlePostureRequest(
    const rm_competition_interfaces::msg::PostureRequest::SharedPtr message)
  {
    if (message->requested_posture.value >
      rm_competition_interfaces::msg::RobotPosture::ENHANCED_MOVE)
    {
      RCLCPP_WARN(
        get_logger(), "Rejecting invalid posture value %u",
        static_cast<unsigned int>(message->requested_posture.value));
      return;
    }

    v2::PostureRequest request;
    request.command_id = message->command_id;
    request.requester_boot_id = local_boot_id_;
    request.requested_posture = static_cast<v2::Posture>(message->requested_posture.value);
    request.valid = message->valid;
    request.request_flags = message->request_flags;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (latest_posture_request_.has_value() &&
        latest_posture_request_->command_id == request.command_id &&
        latest_posture_request_->requester_boot_id == request.requester_boot_id &&
        latest_posture_request_->requested_posture != request.requested_posture)
      {
        RCLCPP_WARN(
          get_logger(), "Rejecting command_id %u reused for a different posture", request.command_id);
        return;
      }
      latest_posture_request_ = request;
    }
    sendPostureRequest(request);
  }

  void sendChassisCommand()
  {
    v2::ChassisCommand command;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      command = latest_chassis_;
      const auto age = std::chrono::duration<double>(steadyNow() - last_cmd_vel_time_).count();
      if (age > cmd_vel_timeout_sec_) {
        command = v2::ChassisCommand{};
      }
    }
    if (!remoteSupports(v2::CapabilityChassisCommand)) {
      command = v2::ChassisCommand{};
    }
    sendMessage(v2::MessageType::ChassisCommand, chassis_sequence_++, command);
  }

  void resendPostureRequest()
  {
    std::optional<v2::PostureRequest> request;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      request = latest_posture_request_;
    }
    if (request.has_value()) {
      sendPostureRequest(*request);
    }
  }

  void sendPostureRequest(const v2::PostureRequest & request)
  {
    if (!remoteSupports(v2::CapabilityPosture)) {
      return;
    }
    sendMessage(v2::MessageType::PostureRequest, posture_sequence_++, request);
  }

  void sendHeartbeat()
  {
    const auto uptime = std::chrono::duration_cast<std::chrono::milliseconds>(
      steadyNow() - start_time_).count();
    v2::Heartbeat heartbeat;
    heartbeat.uptime_ms = static_cast<std::uint32_t>(uptime);
    heartbeat.boot_id = local_boot_id_;
    heartbeat.capabilities =
      v2::CapabilityChassisCommand | v2::CapabilityPosture |
      v2::CapabilityGimbalState | v2::CapabilityRefereeState |
      v2::CapabilityOperatorNavigationTarget;
    heartbeat.ready = true;
    sendMessage(v2::MessageType::Heartbeat, heartbeat_sequence_++, heartbeat);
  }

  template<typename MessageT>
  void sendMessage(v2::MessageType type, std::uint16_t sequence, const MessageT & message)
  {
    const auto frame = v2::encode_message(type, sequence, message);
    if (!frame.has_value()) {
      RCLCPP_WARN(get_logger(), "Failed to encode competition_v2 message type 0x%02x",
        static_cast<unsigned int>(type));
      return;
    }
    writeBytes(*frame);
  }

  void writeBytes(const std::vector<std::uint8_t> & bytes)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (dry_run_) {
      std_msgs::msg::UInt8MultiArray message;
      message.data = bytes;
      mock_tx_pub_->publish(message);
      return;
    }
    std::string error;
    if (!serial_->writeAll(bytes, error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to write competition_v2 frame: %s",
        error.c_str());
    }
  }

  void readSerial()
  {
    std::array<std::uint8_t, 512> bytes{};
    for (int attempt = 0; attempt < 8; ++attempt) {
      std::string error;
      const auto count = serial_->readSome(bytes.data(), bytes.size(), error);
      if (count < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Failed to read competition_v2 frame: %s",
          error.c_str());
        return;
      }
      if (count == 0) {
        break;
      }
      decoder_.append(bytes.data(), static_cast<std::size_t>(count));
    }
    processFrames();
  }

  void processFrames()
  {
    v2::Frame frame;
    while (decoder_.pop(frame)) {
      ++valid_frame_count_;
      last_valid_frame_ros_stamp_ = now();
      last_rx_sequence_ = frame.sequence;
      if (!v2::is_known_message_type(frame.message_type)) {
        ++unknown_message_count_;
        continue;
      }
      switch (static_cast<v2::MessageType>(frame.message_type)) {
        case v2::MessageType::Heartbeat: handleHeartbeat(frame); break;
        case v2::MessageType::PostureState: handlePostureState(frame); break;
        case v2::MessageType::GimbalState: handleGimbalState(frame); break;
        case v2::MessageType::RefereeState: handleRefereeState(frame); break;
        case v2::MessageType::OperatorNavigationTarget: handleOperatorTarget(frame); break;
        case v2::MessageType::ChassisCommand:
        case v2::MessageType::PostureRequest:
          RCLCPP_DEBUG(get_logger(), "Ignoring upper-to-lower message echoed by peer");
          break;
      }
    }
  }

  void handleHeartbeat(const v2::Frame & frame)
  {
    const auto heartbeat = v2::decode_heartbeat(frame);
    if (!heartbeat.has_value()) {
      return;
    }
    const auto receive_stamp = now();
    if (have_remote_heartbeat_ && remote_boot_id_ != heartbeat->boot_id) {
      publishInvalidPosture("lower-controller restart");
      publishInvalidGimbal("lower-controller restart");
    }
    remote_boot_id_ = heartbeat->boot_id;
    remote_uptime_ms_ = heartbeat->uptime_ms;
    remote_capabilities_ = heartbeat->capabilities;
    remote_ready_ = heartbeat->ready;
    heartbeat_clock_offset_ns_ = receive_stamp.nanoseconds() -
      static_cast<std::int64_t>(heartbeat->uptime_ms) * 1000000LL;
    have_clock_sync_ = true;
    have_remote_heartbeat_ = true;
    last_heartbeat_time_ = steadyNow();
  }

  void handlePostureState(const v2::Frame & frame)
  {
    const auto state = v2::decode_posture_state(frame);
    if (!state.has_value()) {
      return;
    }
    rm_competition_interfaces::msg::PostureState message;
    message.header.stamp = now();
    message.posture_target.value = static_cast<std::uint8_t>(state->posture_target);
    message.posture_actual.value = static_cast<std::uint8_t>(state->posture_actual);
    message.ack_command_id = state->ack_command_id;
    message.ack_requester_boot_id = state->ack_requester_boot_id;
    std::optional<v2::PostureRequest> request;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      request = latest_posture_request_;
    }
    message.transitioning = state->transitioning;
    message.fault = state->fault;
    message.fault_code = state->fault_code;
    message.online = state->online && remoteSupports(v2::CapabilityPosture);
    message.valid = state->valid && message.online;
    if (request.has_value()) {
      message.ack_matches_request =
        state->ack_requester_boot_id == local_boot_id_ &&
        state->ack_command_id == request->command_id &&
        state->posture_target == request->requested_posture;
      message.completed = message.valid && message.ack_matches_request && request->valid &&
        !message.transitioning && !message.fault &&
        state->posture_actual == request->requested_posture;
    }
    posture_state_pub_->publish(message);
    last_posture_time_ = steadyNow();
    posture_invalid_published_ = false;
  }

  void handleGimbalState(const v2::Frame & frame)
  {
    const auto state = v2::decode_gimbal_state(frame);
    if (!state.has_value()) {
      return;
    }
    rm_competition_interfaces::msg::GimbalState message;
    message.header.stamp = estimateMcuStamp(state->mcu_time_ms);
    message.relative_yaw_rad = state->relative_yaw_rad;
    message.yaw_rate_rad_s = state->yaw_rate_rad_s;
    message.sample_sequence = state->sample_sequence;
    message.mcu_time_ms = state->mcu_time_ms;
    message.online = state->online && remoteSupports(v2::CapabilityGimbalState);
    message.valid = state->valid && message.online && have_clock_sync_;
    gimbal_state_pub_->publish(message);
    last_gimbal_time_ = steadyNow();
    gimbal_invalid_published_ = false;
  }

  void handleRefereeState(const v2::Frame & frame)
  {
    const auto state = v2::decode_referee_state(frame);
    if (!state.has_value()) {
      return;
    }
    latest_robot_id_ = state->robot_id;
    rm_competition_interfaces::msg::RefereeState message;
    message.header.stamp = now();
    message.valid = state->valid && remoteSupports(v2::CapabilityRefereeState) &&
      state->robot_id != 0U;
    message.game_progress = state->game_progress;
    message.stage_remain_time = state->stage_remain_time;
    message.robot_id = state->robot_id;
    message.current_hp = state->current_hp;
    const bool blue = state->robot_id >= 100U;
    message.self_outpost_hp = blue ? state->blue_outpost_hp : state->red_outpost_hp;
    message.enemy_outpost_hp = blue ? state->red_outpost_hp : state->blue_outpost_hp;
    message.projectile_allowance_17mm = state->projectile_allowance_17mm;
    message.remaining_gold_coin = state->remaining_gold_coin;
    message.diagnostic_flags = state->diagnostic_flags;
    referee_pub_->publish(message);
  }

  void handleOperatorTarget(const v2::Frame & frame)
  {
    const auto target = v2::decode_operator_navigation_target(frame);
    if (!target.has_value()) {
      return;
    }
    rm_competition_interfaces::msg::OperatorNavigationTarget message;
    message.header.stamp = estimateMcuStamp(target->sample_time_ms);
    message.target_x = target->x_m;
    message.target_y = target->y_m;
    message.target_yaw = target->yaw_rad;
    message.command_id = target->command_id;
    message.robot_id = latest_robot_id_;
    message.coordinate_system = static_cast<std::uint8_t>(target->coordinate_frame);
    message.alliance = static_cast<std::uint8_t>(target->alliance);
    message.has_yaw = target->has_yaw;
    message.mcu_sample_time_ms = target->sample_time_ms;
    message.ttl_ms = target->ttl_ms;
    message.transport_valid = target->valid &&
      remoteSupports(v2::CapabilityOperatorNavigationTarget) && have_clock_sync_;
    operator_goal_pub_->publish(message);
  }

  builtin_interfaces::msg::Time estimateMcuStamp(std::uint32_t mcu_time_ms) const
  {
    if (!have_clock_sync_) {
      return now();
    }
    const auto nanoseconds = heartbeat_clock_offset_ns_ +
      static_cast<std::int64_t>(mcu_time_ms) * 1000000LL;
    return rclcpp::Time(nanoseconds, get_clock()->get_clock_type());
  }

  bool connectionOnline() const
  {
    if (!have_remote_heartbeat_ || !remote_ready_) {
      return false;
    }
    return std::chrono::duration<double>(steadyNow() - last_heartbeat_time_).count() <=
           connection_timeout_sec_;
  }

  bool remoteSupports(std::uint32_t capability) const
  {
    return connectionOnline() && (remote_capabilities_ & capability) == capability;
  }

  void publishHealth()
  {
    rm_competition_interfaces::msg::SerialConnectionState state;
    state.header.stamp = now();
    state.online = connectionOnline();
    state.protocol_version = v2::kProtocolVersion;
    state.local_boot_id = local_boot_id_;
    state.remote_capabilities = remote_capabilities_;
    state.remote_boot_id = remote_boot_id_;
    state.remote_uptime_ms = remote_uptime_ms_;
    state.last_rx_sequence = last_rx_sequence_;
    state.last_valid_frame_stamp = last_valid_frame_ros_stamp_;
    state.valid_frame_count = valid_frame_count_;
    state.crc_error_count = decoder_.crc_error_count();
    state.framing_error_count = decoder_.framing_error_count();
    state.unknown_message_count = unknown_message_count_;
    state.compatible = state.online &&
      (remote_capabilities_ & required_remote_capabilities_) == required_remote_capabilities_;
    connection_pub_->publish(state);

    if ((!state.online || ageSeconds(last_gimbal_time_) > gimbal_timeout_sec_) &&
      !gimbal_invalid_published_)
    {
      publishInvalidGimbal(state.online ? "stale" : "serial offline");
    }
    if ((!state.online || ageSeconds(last_posture_time_) > posture_timeout_sec_) &&
      !posture_invalid_published_)
    {
      publishInvalidPosture(state.online ? "stale" : "serial offline");
    }
  }

  double ageSeconds(const SteadyTime & stamp) const
  {
    return std::chrono::duration<double>(steadyNow() - stamp).count();
  }

  void publishInvalidGimbal(const char * reason)
  {
    rm_competition_interfaces::msg::GimbalState message;
    message.header.stamp = now();
    message.online = remoteSupports(v2::CapabilityGimbalState);
    message.valid = false;
    gimbal_state_pub_->publish(message);
    gimbal_invalid_published_ = true;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Gimbal state invalidated: %s", reason);
  }

  void publishInvalidPosture(const char * reason)
  {
    rm_competition_interfaces::msg::PostureState message;
    message.header.stamp = now();
    message.online = remoteSupports(v2::CapabilityPosture);
    message.valid = false;
    posture_state_pub_->publish(message);
    posture_invalid_published_ = true;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Posture state invalidated: %s", reason);
  }

  std::string device_;
  int baudrate_;
  bool dry_run_;
  std::string cmd_vel_topic_;
  std::string posture_request_topic_;
  std::string posture_state_topic_;
  std::string gimbal_state_topic_;
  std::string referee_raw_topic_;
  std::string operator_goal_raw_topic_;
  std::string connection_state_topic_;
  std::string mock_tx_topic_;
  std::string mock_rx_topic_;
  double chassis_rate_hz_;
  double posture_rate_hz_;
  double heartbeat_rate_hz_;
  double read_poll_rate_hz_;
  double health_rate_hz_;
  double cmd_vel_timeout_sec_;
  double connection_timeout_sec_;
  double gimbal_timeout_sec_;
  double posture_timeout_sec_;
  double max_vx_;
  double max_vy_;
  double max_wz_;
  std::uint32_t required_remote_capabilities_;
  const std::uint32_t local_boot_id_;

  mutable std::mutex state_mutex_;
  std::mutex write_mutex_;
  v2::ChassisCommand latest_chassis_;
  std::optional<v2::PostureRequest> latest_posture_request_;
  SteadyTime last_cmd_vel_time_;
  SteadyTime last_heartbeat_time_;
  SteadyTime last_gimbal_time_;
  SteadyTime last_posture_time_;
  SteadyTime start_time_;
  bool have_remote_heartbeat_ = false;
  bool remote_ready_ = false;
  bool have_clock_sync_ = false;
  bool gimbal_invalid_published_ = false;
  bool posture_invalid_published_ = false;
  std::int64_t heartbeat_clock_offset_ns_ = 0;
  std::uint32_t remote_capabilities_ = 0;
  std::uint32_t remote_boot_id_ = 0;
  std::uint32_t remote_uptime_ms_ = 0;
  std::uint8_t latest_robot_id_ = 0;
  std::uint16_t last_rx_sequence_ = 0;
  std::uint16_t chassis_sequence_ = 0;
  std::uint16_t posture_sequence_ = 0;
  std::uint16_t heartbeat_sequence_ = 0;
  std::uint64_t valid_frame_count_ = 0;
  std::uint64_t unknown_message_count_ = 0;
  builtin_interfaces::msg::Time last_valid_frame_ros_stamp_;

  std::unique_ptr<PosixSerialPort> serial_;
  v2::StreamDecoder decoder_;
  rclcpp::Publisher<rm_competition_interfaces::msg::PostureState>::SharedPtr posture_state_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::GimbalState>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::RefereeState>::SharedPtr referee_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::OperatorNavigationTarget>::SharedPtr
    operator_goal_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::SerialConnectionState>::SharedPtr
    connection_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr mock_tx_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::PostureRequest>::SharedPtr
    posture_request_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr mock_rx_sub_;
  rclcpp::TimerBase::SharedPtr chassis_timer_;
  rclcpp::TimerBase::SharedPtr posture_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompetitionV2TransportNode>());
  rclcpp::shutdown();
  return 0;
}
