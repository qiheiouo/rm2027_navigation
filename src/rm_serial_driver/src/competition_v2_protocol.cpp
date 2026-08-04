#include "rm_serial_driver/competition_v2_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

namespace rm_serial_driver
{
namespace competition_v2
{
namespace
{

constexpr std::size_t kChassisCommandPayloadSize = 13;
constexpr std::size_t kPostureRequestPayloadSize = 11;
constexpr std::size_t kPostureStatePayloadSize = 13;
constexpr std::size_t kGimbalStatePayloadSize = 17;
constexpr std::size_t kChassisHeadingStatePayloadSize = 19;
constexpr std::size_t kRefereeStatePayloadSize = 17;
constexpr std::size_t kOperatorTargetPayloadSize = 25;
constexpr std::size_t kHeartbeatPayloadSize = 13;
constexpr float kPi = 3.14159265358979323846F;

constexpr std::uint8_t kEnabledFlag = 1U << 0U;
constexpr std::uint8_t kPostureValidFlag = 1U << 0U;
constexpr std::uint8_t kPostureOnlineFlag = 1U << 1U;
constexpr std::uint8_t kPostureTransitioningFlag = 1U << 2U;
constexpr std::uint8_t kPostureFaultFlag = 1U << 3U;
constexpr std::uint8_t kOperatorValidFlag = 1U << 0U;
constexpr std::uint8_t kOperatorHasYawFlag = 1U << 1U;

void append_u16(std::vector<std::uint8_t> & output, std::uint16_t value)
{
  output.push_back(static_cast<std::uint8_t>(value & 0xffU));
  output.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
}

void append_u32(std::vector<std::uint8_t> & output, std::uint32_t value)
{
  output.push_back(static_cast<std::uint8_t>(value & 0xffU));
  output.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
  output.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xffU));
  output.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xffU));
}

void append_float(std::vector<std::uint8_t> & output, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  static_assert(std::numeric_limits<float>::is_iec559);
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(output, bits);
}

std::uint16_t read_u16(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[0]) |
    (static_cast<std::uint16_t>(data[1]) << 8U));
}

std::uint32_t read_u32(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

float read_float(const std::uint8_t * data)
{
  const std::uint32_t bits = read_u32(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool finite(float value)
{
  return std::isfinite(value);
}

bool valid_wrapped_yaw(float yaw)
{
  return finite(yaw) && yaw >= -kPi && yaw < kPi;
}

bool frame_matches(const Frame & frame, MessageType type, std::size_t payload_size)
{
  return frame.version == kProtocolVersion &&
         frame.message_type == static_cast<std::uint8_t>(type) &&
         frame.payload.size() == payload_size;
}

}  // namespace

bool is_known_message_type(std::uint8_t value)
{
  switch (static_cast<MessageType>(value)) {
    case MessageType::ChassisCommand:
    case MessageType::PostureRequest:
    case MessageType::Heartbeat:
    case MessageType::PostureState:
    case MessageType::GimbalState:
    case MessageType::RefereeState:
    case MessageType::OperatorNavigationTarget:
    case MessageType::ChassisHeadingState:
      return true;
  }
  return false;
}

bool is_valid_posture(Posture value)
{
  return static_cast<std::uint8_t>(value) <=
         static_cast<std::uint8_t>(Posture::EnhancedMove);
}

bool is_valid_coordinate_frame(CoordinateFrame value)
{
  return static_cast<std::uint8_t>(value) <= static_cast<std::uint8_t>(CoordinateFrame::Map);
}

bool is_valid_alliance(Alliance value)
{
  return static_cast<std::uint8_t>(value) <= static_cast<std::uint8_t>(Alliance::Blue);
}

bool is_newer_sequence(std::uint16_t candidate, std::uint16_t reference)
{
  const auto delta = static_cast<std::uint16_t>(candidate - reference);
  return delta != 0U && delta < 0x8000U;
}

bool is_newer_command_id(std::uint32_t candidate, std::uint32_t reference)
{
  const auto delta = static_cast<std::uint32_t>(candidate - reference);
  return delta != 0U && delta < 0x80000000U;
}

std::uint16_t crc16_modbus(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xffffU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ?
        static_cast<std::uint16_t>((crc >> 1U) ^ 0xa001U) :
        static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

std::optional<std::vector<std::uint8_t>> encode_frame(const Frame & frame)
{
  if (frame.version != kProtocolVersion || frame.payload.size() > kMaxPayloadSize) {
    return std::nullopt;
  }

  std::vector<std::uint8_t> output;
  output.reserve(kHeaderSize + frame.payload.size() + kCrcSize);
  output.push_back(kMagic0);
  output.push_back(kMagic1);
  output.push_back(frame.version);
  output.push_back(frame.message_type);
  append_u16(output, static_cast<std::uint16_t>(frame.payload.size()));
  append_u16(output, frame.sequence);
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());
  const auto crc = crc16_modbus(output.data() + 2, output.size() - 2);
  append_u16(output, crc);
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(const ChassisCommand & message)
{
  if (!finite(message.vx_mps) || !finite(message.vy_mps) || !finite(message.wz_rad_s)) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> output;
  output.reserve(kChassisCommandPayloadSize);
  append_float(output, message.vx_mps);
  append_float(output, message.vy_mps);
  append_float(output, message.wz_rad_s);
  output.push_back(message.enabled ? kEnabledFlag : 0U);
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(const PostureRequest & message)
{
  if (!is_valid_posture(message.requested_posture)) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> output;
  output.reserve(kPostureRequestPayloadSize);
  append_u32(output, message.command_id);
  append_u32(output, message.requester_boot_id);
  output.push_back(static_cast<std::uint8_t>(message.requested_posture));
  output.push_back(message.valid ? kEnabledFlag : 0U);
  output.push_back(message.request_flags);
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(const PostureState & message)
{
  if (!is_valid_posture(message.posture_target) || !is_valid_posture(message.posture_actual)) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> output;
  output.reserve(kPostureStatePayloadSize);
  append_u32(output, message.ack_command_id);
  append_u32(output, message.ack_requester_boot_id);
  output.push_back(static_cast<std::uint8_t>(message.posture_target));
  output.push_back(static_cast<std::uint8_t>(message.posture_actual));
  output.push_back(
    (message.valid ? kPostureValidFlag : 0U) |
    (message.online ? kPostureOnlineFlag : 0U) |
    (message.transitioning ? kPostureTransitioningFlag : 0U) |
    (message.fault ? kPostureFaultFlag : 0U));
  append_u16(output, message.fault_code);
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(const GimbalState & message)
{
  if (!valid_wrapped_yaw(message.relative_yaw_rad) || !finite(message.yaw_rate_rad_s)) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> output;
  output.reserve(kGimbalStatePayloadSize);
  append_float(output, message.relative_yaw_rad);
  append_float(output, message.yaw_rate_rad_s);
  append_u32(output, message.sample_sequence);
  append_u32(output, message.mcu_time_ms);
  output.push_back(
    (message.valid ? kPostureValidFlag : 0U) |
    (message.online ? kPostureOnlineFlag : 0U));
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(
  const ChassisHeadingState & message)
{
  if (!valid_wrapped_yaw(message.yaw_rad) || !finite(message.yaw_rate_rad_s)) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> output;
  output.reserve(kChassisHeadingStatePayloadSize);
  append_float(output, message.yaw_rad);
  append_float(output, message.yaw_rate_rad_s);
  append_u32(output, message.sample_sequence);
  append_u32(output, message.mcu_time_ms);
  append_u16(output, message.reset_counter);
  output.push_back(
    (message.valid ? kPostureValidFlag : 0U) |
    (message.online ? kPostureOnlineFlag : 0U));
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(const RefereeState & message)
{
  std::vector<std::uint8_t> output;
  output.reserve(kRefereeStatePayloadSize);
  output.push_back(message.game_progress);
  append_u16(output, message.stage_remain_time);
  output.push_back(message.robot_id);
  append_u16(output, message.current_hp);
  append_u16(output, message.red_outpost_hp);
  append_u16(output, message.blue_outpost_hp);
  append_u16(output, message.projectile_allowance_17mm);
  append_u16(output, message.remaining_gold_coin);
  append_u16(output, message.diagnostic_flags);
  output.push_back(message.valid ? kEnabledFlag : 0U);
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(
  const OperatorNavigationTarget & message)
{
  if (!is_valid_coordinate_frame(message.coordinate_frame) || !is_valid_alliance(message.alliance) ||
    !finite(message.x_m) || !finite(message.y_m) ||
    (message.has_yaw && !valid_wrapped_yaw(message.yaw_rad)))
  {
    return std::nullopt;
  }
  std::vector<std::uint8_t> output;
  output.reserve(kOperatorTargetPayloadSize);
  append_u32(output, message.command_id);
  output.push_back(static_cast<std::uint8_t>(message.coordinate_frame));
  output.push_back(static_cast<std::uint8_t>(message.alliance));
  output.push_back(
    (message.valid ? kOperatorValidFlag : 0U) |
    (message.has_yaw ? kOperatorHasYawFlag : 0U));
  append_float(output, message.x_m);
  append_float(output, message.y_m);
  append_float(output, message.has_yaw ? message.yaw_rad : 0.0F);
  append_u32(output, message.sample_time_ms);
  append_u16(output, message.ttl_ms);
  return output;
}

std::optional<std::vector<std::uint8_t>> encode_payload(const Heartbeat & message)
{
  std::vector<std::uint8_t> output;
  output.reserve(kHeartbeatPayloadSize);
  append_u32(output, message.uptime_ms);
  append_u32(output, message.boot_id);
  append_u32(output, message.capabilities);
  output.push_back(message.ready ? kEnabledFlag : 0U);
  return output;
}

std::optional<ChassisCommand> decode_chassis_command(const Frame & frame)
{
  if (!frame_matches(frame, MessageType::ChassisCommand, kChassisCommandPayloadSize)) {
    return std::nullopt;
  }
  ChassisCommand message;
  message.vx_mps = read_float(frame.payload.data());
  message.vy_mps = read_float(frame.payload.data() + 4);
  message.wz_rad_s = read_float(frame.payload.data() + 8);
  message.enabled = (frame.payload[12] & kEnabledFlag) != 0U;
  if (!finite(message.vx_mps) || !finite(message.vy_mps) || !finite(message.wz_rad_s)) {
    return std::nullopt;
  }
  return message;
}

std::optional<PostureRequest> decode_posture_request(const Frame & frame)
{
  if (!frame_matches(frame, MessageType::PostureRequest, kPostureRequestPayloadSize)) {
    return std::nullopt;
  }
  PostureRequest message;
  message.command_id = read_u32(frame.payload.data());
  message.requester_boot_id = read_u32(frame.payload.data() + 4);
  message.requested_posture = static_cast<Posture>(frame.payload[8]);
  message.valid = (frame.payload[9] & kEnabledFlag) != 0U;
  message.request_flags = frame.payload[10];
  return is_valid_posture(message.requested_posture) ?
         std::optional<PostureRequest>(message) : std::nullopt;
}

std::optional<PostureState> decode_posture_state(const Frame & frame)
{
  if (!frame_matches(frame, MessageType::PostureState, kPostureStatePayloadSize)) {
    return std::nullopt;
  }
  PostureState message;
  message.ack_command_id = read_u32(frame.payload.data());
  message.ack_requester_boot_id = read_u32(frame.payload.data() + 4);
  message.posture_target = static_cast<Posture>(frame.payload[8]);
  message.posture_actual = static_cast<Posture>(frame.payload[9]);
  const auto flags = frame.payload[10];
  message.valid = (flags & kPostureValidFlag) != 0U;
  message.online = (flags & kPostureOnlineFlag) != 0U;
  message.transitioning = (flags & kPostureTransitioningFlag) != 0U;
  message.fault = (flags & kPostureFaultFlag) != 0U;
  message.fault_code = read_u16(frame.payload.data() + 11);
  if (!is_valid_posture(message.posture_target) || !is_valid_posture(message.posture_actual)) {
    return std::nullopt;
  }
  return message;
}

std::optional<GimbalState> decode_gimbal_state(const Frame & frame)
{
  if (!frame_matches(frame, MessageType::GimbalState, kGimbalStatePayloadSize)) {
    return std::nullopt;
  }
  GimbalState message;
  message.relative_yaw_rad = read_float(frame.payload.data());
  message.yaw_rate_rad_s = read_float(frame.payload.data() + 4);
  message.sample_sequence = read_u32(frame.payload.data() + 8);
  message.mcu_time_ms = read_u32(frame.payload.data() + 12);
  message.valid = (frame.payload[16] & kPostureValidFlag) != 0U;
  message.online = (frame.payload[16] & kPostureOnlineFlag) != 0U;
  if (!valid_wrapped_yaw(message.relative_yaw_rad) || !finite(message.yaw_rate_rad_s)) {
    return std::nullopt;
  }
  return message;
}

std::optional<ChassisHeadingState> decode_chassis_heading_state(const Frame & frame)
{
  if (!frame_matches(
      frame, MessageType::ChassisHeadingState, kChassisHeadingStatePayloadSize))
  {
    return std::nullopt;
  }
  ChassisHeadingState message;
  message.yaw_rad = read_float(frame.payload.data());
  message.yaw_rate_rad_s = read_float(frame.payload.data() + 4);
  message.sample_sequence = read_u32(frame.payload.data() + 8);
  message.mcu_time_ms = read_u32(frame.payload.data() + 12);
  message.reset_counter = read_u16(frame.payload.data() + 16);
  message.valid = (frame.payload[18] & kPostureValidFlag) != 0U;
  message.online = (frame.payload[18] & kPostureOnlineFlag) != 0U;
  if (!valid_wrapped_yaw(message.yaw_rad) || !finite(message.yaw_rate_rad_s)) {
    return std::nullopt;
  }
  return message;
}

std::optional<RefereeState> decode_referee_state(const Frame & frame)
{
  if (!frame_matches(frame, MessageType::RefereeState, kRefereeStatePayloadSize)) {
    return std::nullopt;
  }
  RefereeState message;
  message.game_progress = frame.payload[0];
  message.stage_remain_time = read_u16(frame.payload.data() + 1);
  message.robot_id = frame.payload[3];
  message.current_hp = read_u16(frame.payload.data() + 4);
  message.red_outpost_hp = read_u16(frame.payload.data() + 6);
  message.blue_outpost_hp = read_u16(frame.payload.data() + 8);
  message.projectile_allowance_17mm = read_u16(frame.payload.data() + 10);
  message.remaining_gold_coin = read_u16(frame.payload.data() + 12);
  message.diagnostic_flags = read_u16(frame.payload.data() + 14);
  message.valid = (frame.payload[16] & kEnabledFlag) != 0U;
  return message;
}

std::optional<OperatorNavigationTarget> decode_operator_navigation_target(const Frame & frame)
{
  if (!frame_matches(frame, MessageType::OperatorNavigationTarget, kOperatorTargetPayloadSize)) {
    return std::nullopt;
  }
  OperatorNavigationTarget message;
  message.command_id = read_u32(frame.payload.data());
  message.coordinate_frame = static_cast<CoordinateFrame>(frame.payload[4]);
  message.alliance = static_cast<Alliance>(frame.payload[5]);
  const auto flags = frame.payload[6];
  message.valid = (flags & kOperatorValidFlag) != 0U;
  message.has_yaw = (flags & kOperatorHasYawFlag) != 0U;
  message.x_m = read_float(frame.payload.data() + 7);
  message.y_m = read_float(frame.payload.data() + 11);
  message.yaw_rad = read_float(frame.payload.data() + 15);
  message.sample_time_ms = read_u32(frame.payload.data() + 19);
  message.ttl_ms = read_u16(frame.payload.data() + 23);
  if (!is_valid_coordinate_frame(message.coordinate_frame) || !is_valid_alliance(message.alliance) ||
    !finite(message.x_m) || !finite(message.y_m) ||
    (message.has_yaw && !valid_wrapped_yaw(message.yaw_rad)))
  {
    return std::nullopt;
  }
  return message;
}

std::optional<Heartbeat> decode_heartbeat(const Frame & frame)
{
  if (!frame_matches(frame, MessageType::Heartbeat, kHeartbeatPayloadSize)) {
    return std::nullopt;
  }
  Heartbeat message;
  message.uptime_ms = read_u32(frame.payload.data());
  message.boot_id = read_u32(frame.payload.data() + 4);
  message.capabilities = read_u32(frame.payload.data() + 8);
  message.ready = (frame.payload[12] & kEnabledFlag) != 0U;
  return message;
}

void StreamDecoder::append(const std::uint8_t * data, std::size_t size)
{
  if (data != nullptr && size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
    if (buffer_.size() > kMaxStreamBufferSize) {
      const auto overflow = buffer_.size() - kMaxStreamBufferSize;
      framing_error_count_ += overflow;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(overflow));
    }
  }
}

void StreamDecoder::append(const std::vector<std::uint8_t> & data)
{
  append(data.data(), data.size());
}

bool StreamDecoder::pop(Frame & frame)
{
  while (true) {
    if (buffer_.size() < 2U) {
      return false;
    }

    std::size_t magic_index = 0;
    while (magic_index + 1U < buffer_.size() &&
      (buffer_[magic_index] != kMagic0 || buffer_[magic_index + 1U] != kMagic1))
    {
      ++magic_index;
    }
    if (magic_index + 1U >= buffer_.size()) {
      const bool keep_last = buffer_.back() == kMagic0;
      framing_error_count_ += buffer_.size() - (keep_last ? 1U : 0U);
      const auto last = buffer_.back();
      buffer_.clear();
      if (keep_last) {
        buffer_.push_back(last);
      }
      return false;
    }
    if (magic_index > 0U) {
      framing_error_count_ += magic_index;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(magic_index));
    }
    if (buffer_.size() < kHeaderSize) {
      return false;
    }

    const std::uint8_t version = buffer_[2];
    const std::size_t payload_size = read_u16(buffer_.data() + 4);
    if (version != kProtocolVersion || payload_size > kMaxPayloadSize) {
      ++framing_error_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = kHeaderSize + payload_size + kCrcSize;
    if (buffer_.size() < frame_size) {
      return false;
    }
    const auto expected_crc = crc16_modbus(buffer_.data() + 2, 6U + payload_size);
    const auto received_crc = read_u16(buffer_.data() + kHeaderSize + payload_size);
    if (expected_crc != received_crc) {
      ++crc_error_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    frame.version = version;
    frame.message_type = buffer_[3];
    frame.sequence = read_u16(buffer_.data() + 6);
    frame.payload.assign(
      buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
      buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + payload_size));
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    return true;
  }
}

std::size_t StreamDecoder::buffered_size() const
{
  return buffer_.size();
}

void StreamDecoder::clear()
{
  buffer_.clear();
}

std::uint64_t StreamDecoder::crc_error_count() const
{
  return crc_error_count_;
}

std::uint64_t StreamDecoder::framing_error_count() const
{
  return framing_error_count_;
}

}  // namespace competition_v2
}  // namespace rm_serial_driver
