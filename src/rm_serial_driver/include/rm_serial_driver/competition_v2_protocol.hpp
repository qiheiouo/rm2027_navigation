#ifndef RM_SERIAL_DRIVER__COMPETITION_V2_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER__COMPETITION_V2_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace rm_serial_driver
{
namespace competition_v2
{

constexpr std::uint8_t kMagic0 = 0x52;  // 'R'
constexpr std::uint8_t kMagic1 = 0x4d;  // 'M'
constexpr std::uint8_t kProtocolVersion = 2;
constexpr std::size_t kHeaderSize = 8;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kMaxPayloadSize = 128;
constexpr std::size_t kMaxStreamBufferSize = 4096;

enum class MessageType : std::uint8_t
{
  ChassisCommand = 0x01,
  PostureRequest = 0x02,
  Heartbeat = 0x03,
  PostureState = 0x81,
  GimbalState = 0x82,
  RefereeState = 0x83,
  OperatorNavigationTarget = 0x84,
};

enum class Posture : std::uint8_t
{
  Attack = 0,
  Move = 1,
  Defense = 2,
  EnhancedAttack = 3,
  EnhancedDefense = 4,
  EnhancedMove = 5,
};

enum class CoordinateFrame : std::uint8_t
{
  Unknown = 0,
  RefereeField = 1,
  Map = 2,
};

enum class Alliance : std::uint8_t
{
  Unknown = 0,
  Red = 1,
  Blue = 2,
};

enum Capability : std::uint32_t
{
  CapabilityChassisCommand = 1U << 0U,
  CapabilityPosture = 1U << 1U,
  CapabilityGimbalState = 1U << 2U,
  CapabilityRefereeState = 1U << 3U,
  CapabilityOperatorNavigationTarget = 1U << 4U,
};

struct Frame
{
  std::uint8_t version = kProtocolVersion;
  std::uint8_t message_type = 0;
  std::uint16_t sequence = 0;
  std::vector<std::uint8_t> payload;
};

struct ChassisCommand
{
  float vx_mps = 0.0F;
  float vy_mps = 0.0F;
  float wz_rad_s = 0.0F;
  bool enabled = false;
};

struct PostureRequest
{
  std::uint32_t command_id = 0;
  std::uint32_t requester_boot_id = 0;
  Posture requested_posture = Posture::Move;
  bool valid = false;
  std::uint8_t request_flags = 0;
};

struct PostureState
{
  std::uint32_t ack_command_id = 0;
  std::uint32_t ack_requester_boot_id = 0;
  Posture posture_target = Posture::Move;
  Posture posture_actual = Posture::Move;
  bool valid = false;
  bool online = false;
  bool transitioning = false;
  bool fault = false;
  std::uint16_t fault_code = 0;
};

struct GimbalState
{
  float relative_yaw_rad = 0.0F;
  float yaw_rate_rad_s = 0.0F;
  std::uint32_t sample_sequence = 0;
  std::uint32_t mcu_time_ms = 0;
  bool valid = false;
  bool online = false;
};

struct RefereeState
{
  std::uint8_t game_progress = 0;
  std::uint16_t stage_remain_time = 0;
  std::uint8_t robot_id = 0;
  std::uint16_t current_hp = 0;
  std::uint16_t red_outpost_hp = 0;
  std::uint16_t blue_outpost_hp = 0;
  std::uint16_t projectile_allowance_17mm = 0;
  std::uint16_t remaining_gold_coin = 0;
  std::uint16_t diagnostic_flags = 0;
  bool valid = false;
};

struct OperatorNavigationTarget
{
  std::uint32_t command_id = 0;
  CoordinateFrame coordinate_frame = CoordinateFrame::Unknown;
  Alliance alliance = Alliance::Unknown;
  float x_m = 0.0F;
  float y_m = 0.0F;
  float yaw_rad = 0.0F;
  std::uint32_t sample_time_ms = 0;
  std::uint16_t ttl_ms = 0;
  bool valid = false;
  bool has_yaw = false;
};

struct Heartbeat
{
  std::uint32_t uptime_ms = 0;
  std::uint32_t boot_id = 0;
  std::uint32_t capabilities = 0;
  bool ready = false;
};

bool is_known_message_type(std::uint8_t value);
bool is_valid_posture(Posture value);
bool is_valid_coordinate_frame(CoordinateFrame value);
bool is_valid_alliance(Alliance value);
bool is_newer_sequence(std::uint16_t candidate, std::uint16_t reference);
bool is_newer_command_id(std::uint32_t candidate, std::uint32_t reference);

std::uint16_t crc16_modbus(const std::uint8_t * data, std::size_t size);
std::optional<std::vector<std::uint8_t>> encode_frame(const Frame & frame);

std::optional<std::vector<std::uint8_t>> encode_payload(const ChassisCommand & message);
std::optional<std::vector<std::uint8_t>> encode_payload(const PostureRequest & message);
std::optional<std::vector<std::uint8_t>> encode_payload(const PostureState & message);
std::optional<std::vector<std::uint8_t>> encode_payload(const GimbalState & message);
std::optional<std::vector<std::uint8_t>> encode_payload(const RefereeState & message);
std::optional<std::vector<std::uint8_t>> encode_payload(
  const OperatorNavigationTarget & message);
std::optional<std::vector<std::uint8_t>> encode_payload(const Heartbeat & message);

std::optional<ChassisCommand> decode_chassis_command(const Frame & frame);
std::optional<PostureRequest> decode_posture_request(const Frame & frame);
std::optional<PostureState> decode_posture_state(const Frame & frame);
std::optional<GimbalState> decode_gimbal_state(const Frame & frame);
std::optional<RefereeState> decode_referee_state(const Frame & frame);
std::optional<OperatorNavigationTarget> decode_operator_navigation_target(const Frame & frame);
std::optional<Heartbeat> decode_heartbeat(const Frame & frame);

template<typename MessageT>
std::optional<std::vector<std::uint8_t>> encode_message(
  MessageType type, std::uint16_t sequence, const MessageT & message)
{
  const auto payload = encode_payload(message);
  if (!payload.has_value()) {
    return std::nullopt;
  }
  return encode_frame(
    Frame{kProtocolVersion, static_cast<std::uint8_t>(type), sequence, *payload});
}

class StreamDecoder
{
public:
  void append(const std::uint8_t * data, std::size_t size);
  void append(const std::vector<std::uint8_t> & data);
  bool pop(Frame & frame);
  std::size_t buffered_size() const;
  void clear();
  std::uint64_t crc_error_count() const;
  std::uint64_t framing_error_count() const;

private:
  std::vector<std::uint8_t> buffer_;
  std::uint64_t crc_error_count_ = 0;
  std::uint64_t framing_error_count_ = 0;
};

}  // namespace competition_v2
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__COMPETITION_V2_PROTOCOL_HPP_
