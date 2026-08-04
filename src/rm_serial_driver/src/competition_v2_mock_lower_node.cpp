#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_serial_driver/competition_v2_protocol.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace v2 = rm_serial_driver::competition_v2;

class CompetitionV2MockLowerNode : public rclcpp::Node
{
public:
  CompetitionV2MockLowerNode()
  : Node("competition_v2_mock_lower_node")
  {
    rx_topic_ = declare_parameter<std::string>("upper_tx_topic", "/serial/v2/mock_tx");
    tx_topic_ = declare_parameter<std::string>("upper_rx_topic", "/serial/v2/mock_rx");
    boot_id_ = static_cast<std::uint32_t>(declare_parameter<std::int64_t>(
      "boot_id", 0x20270001));
    gimbal_yaw_rad_ = declare_parameter<double>("gimbal_yaw_rad", 0.25);
    gimbal_rate_rad_s_ = declare_parameter<double>("gimbal_rate_rad_s", 0.0);
    gimbal_publish_duration_sec_ = declare_parameter<double>(
      "gimbal_publish_duration_sec", -1.0);
    heartbeat_publish_duration_sec_ = declare_parameter<double>(
      "heartbeat_publish_duration_sec", -1.0);
    publish_operator_target_ = declare_parameter<bool>("publish_operator_target", false);
    posture_transition_responses_ = std::max(
      0, static_cast<int>(declare_parameter<int>("posture_transition_responses", 0)));

    tx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(tx_topic_, rclcpp::QoS(100));
    rx_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      rx_topic_, rclcpp::QoS(100),
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
        decoder_.append(message->data);
        processFrames();
      });

    start_time_ = std::chrono::steady_clock::now();
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publishHeartbeat();});
    gimbal_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {publishGimbal();});
    referee_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {publishReferee();});
    if (publish_operator_target_) {
      operator_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() {publishOperatorTarget();});
    }
    RCLCPP_WARN(
      get_logger(), "competition_v2 mock lower controller active; no serial device is opened");
  }

private:
  std::uint32_t uptimeMs() const
  {
    return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time_).count());
  }

  void processFrames()
  {
    v2::Frame frame;
    while (decoder_.pop(frame)) {
      switch (static_cast<v2::MessageType>(frame.message_type)) {
        case v2::MessageType::PostureRequest:
          handlePostureRequest(frame);
          break;
        case v2::MessageType::ChassisCommand:
          latest_chassis_ = v2::decode_chassis_command(frame);
          break;
        case v2::MessageType::Heartbeat:
          latest_upper_heartbeat_ = v2::decode_heartbeat(frame);
          break;
        default:
          break;
      }
    }
  }

  void handlePostureRequest(const v2::Frame & frame)
  {
    const auto request = v2::decode_posture_request(frame);
    if (!request.has_value()) {
      return;
    }
    const bool new_command = !last_posture_request_.has_value() ||
      last_posture_request_->requester_boot_id != request->requester_boot_id ||
      last_posture_request_->command_id != request->command_id;
    if (new_command) {
      last_posture_request_ = request;
      posture_transition_remaining_ = posture_transition_responses_;
    }
    v2::PostureState state;
    state.ack_command_id = request->command_id;
    state.ack_requester_boot_id = request->requester_boot_id;
    state.posture_target = request->requested_posture;
    state.transitioning = request->valid && posture_transition_remaining_ > 0;
    if (state.transitioning) {
      --posture_transition_remaining_;
    } else if (request->valid) {
      posture_actual_ = request->requested_posture;
    }
    state.posture_actual = posture_actual_;
    state.valid = request->valid;
    state.online = true;
    send(v2::MessageType::PostureState, state);
  }

  void publishHeartbeat()
  {
    if (heartbeat_publish_duration_sec_ >= 0.0 &&
      uptimeMs() > static_cast<std::uint32_t>(heartbeat_publish_duration_sec_ * 1000.0))
    {
      return;
    }
    v2::Heartbeat heartbeat;
    heartbeat.uptime_ms = uptimeMs();
    heartbeat.boot_id = boot_id_;
    heartbeat.capabilities =
      v2::CapabilityChassisCommand | v2::CapabilityPosture |
      v2::CapabilityGimbalState | v2::CapabilityRefereeState |
      v2::CapabilityOperatorNavigationTarget;
    heartbeat.ready = true;
    send(v2::MessageType::Heartbeat, heartbeat);
  }

  void publishGimbal()
  {
    if (gimbal_publish_duration_sec_ >= 0.0 &&
      uptimeMs() > static_cast<std::uint32_t>(gimbal_publish_duration_sec_ * 1000.0))
    {
      return;
    }
    v2::GimbalState state;
    state.relative_yaw_rad = static_cast<float>(gimbal_yaw_rad_);
    state.yaw_rate_rad_s = static_cast<float>(gimbal_rate_rad_s_);
    state.sample_sequence = gimbal_sample_sequence_++;
    state.mcu_time_ms = uptimeMs();
    state.valid = true;
    state.online = true;
    send(v2::MessageType::GimbalState, state);
  }

  void publishReferee()
  {
    v2::RefereeState state;
    state.game_progress = 4;
    state.stage_remain_time = 300;
    state.robot_id = 107;
    state.current_hp = 400;
    state.red_outpost_hp = 1200;
    state.blue_outpost_hp = 1100;
    state.projectile_allowance_17mm = 100;
    state.remaining_gold_coin = 25;
    state.diagnostic_flags = 0;
    state.valid = true;
    send(v2::MessageType::RefereeState, state);
  }

  void publishOperatorTarget()
  {
    v2::OperatorNavigationTarget target;
    target.command_id = operator_command_id_++;
    target.coordinate_frame = v2::CoordinateFrame::Map;
    target.x_m = 0.0F;
    target.y_m = 0.0F;
    target.sample_time_ms = uptimeMs();
    target.ttl_ms = 1000;
    target.valid = true;
    send(v2::MessageType::OperatorNavigationTarget, target);
  }

  template<typename MessageT>
  void send(v2::MessageType type, const MessageT & message)
  {
    const auto bytes = v2::encode_message(type, nextSequence(type), message);
    if (!bytes.has_value()) {
      RCLCPP_ERROR(get_logger(), "mock failed to encode message type 0x%02x",
        static_cast<unsigned int>(type));
      return;
    }
    std_msgs::msg::UInt8MultiArray output;
    output.data = *bytes;
    tx_pub_->publish(output);
  }

  std::uint16_t nextSequence(v2::MessageType type)
  {
    switch (type) {
      case v2::MessageType::Heartbeat: return heartbeat_sequence_++;
      case v2::MessageType::PostureState: return posture_sequence_++;
      case v2::MessageType::GimbalState: return gimbal_sequence_++;
      case v2::MessageType::RefereeState: return referee_sequence_++;
      case v2::MessageType::OperatorNavigationTarget: return operator_sequence_++;
      case v2::MessageType::ChassisCommand:
      case v2::MessageType::PostureRequest:
        return 0;
    }
    return 0;
  }

  std::string rx_topic_;
  std::string tx_topic_;
  std::uint32_t boot_id_;
  double gimbal_yaw_rad_;
  double gimbal_rate_rad_s_;
  double gimbal_publish_duration_sec_;
  double heartbeat_publish_duration_sec_;
  bool publish_operator_target_;
  int posture_transition_responses_;
  int posture_transition_remaining_ = 0;
  std::chrono::steady_clock::time_point start_time_;
  v2::Posture posture_actual_ = v2::Posture::Move;
  std::optional<v2::ChassisCommand> latest_chassis_;
  std::optional<v2::Heartbeat> latest_upper_heartbeat_;
  std::optional<v2::PostureRequest> last_posture_request_;
  std::uint16_t heartbeat_sequence_ = 0;
  std::uint16_t posture_sequence_ = 0;
  std::uint16_t gimbal_sequence_ = 0;
  std::uint16_t referee_sequence_ = 0;
  std::uint16_t operator_sequence_ = 0;
  std::uint32_t gimbal_sample_sequence_ = 0;
  std::uint32_t operator_command_id_ = 1;
  v2::StreamDecoder decoder_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_timer_;
  rclcpp::TimerBase::SharedPtr referee_timer_;
  rclcpp::TimerBase::SharedPtr operator_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompetitionV2MockLowerNode>());
  rclcpp::shutdown();
  return 0;
}
