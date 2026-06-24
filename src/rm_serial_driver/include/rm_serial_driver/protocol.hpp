#ifndef RM_SERIAL_DRIVER__PROTOCOL_HPP_
#define RM_SERIAL_DRIVER__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace rm_serial_driver
{
namespace legacy_v1
{

constexpr std::uint8_t kFrameHeader = 0x3e;
constexpr std::size_t kCommandPayloadSize = 17;
constexpr std::size_t kCommandFrameSize = 2 + kCommandPayloadSize;
constexpr std::size_t kMaxPayloadSize = 128;

struct Command
{
  float vx = 0.0F;
  float vy = 0.0F;
  float wz = 0.0F;
  std::uint8_t sequence = 0;
  std::uint8_t navigation_state = 0;
  std::uint8_t remake_command = 0;
  std::uint8_t bullet_command = 0;
  std::uint8_t allow_restart = 0;
};

using CommandFrame = std::array<std::uint8_t, kCommandFrameSize>;

// Legacy V1 has no CRC or checksum. The codec preserves that wire format.
std::optional<CommandFrame> encode_command(const Command & command);
std::optional<Command> decode_command(const CommandFrame & frame);

struct Frame
{
  std::vector<std::uint8_t> payload;
};

class StreamDecoder
{
public:
  void append(const std::uint8_t * data, std::size_t size);
  void append(const std::vector<std::uint8_t> & data);
  bool pop(Frame & frame);
  std::size_t buffered_size() const;
  void clear();

private:
  std::vector<std::uint8_t> buffer_;
};

}  // namespace legacy_v1

namespace hpm_crc_v1
{

using Command = legacy_v1::Command;

constexpr std::size_t kCommandPayloadSize = legacy_v1::kCommandPayloadSize;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kCommandFrameSize =
  2 + kCommandPayloadSize + kCrcSize;
constexpr std::size_t kMaxPayloadSize = legacy_v1::kMaxPayloadSize;

using CommandFrame = std::array<std::uint8_t, kCommandFrameSize>;

// Modbus CRC16 over the payload only: polynomial 0xA001, initial value 0xFFFF.
// The wire format appends the low byte first, matching rm2026_sentry/rm2025_hpm.
std::uint16_t crc16_modbus(const std::uint8_t * data, std::size_t size);
std::optional<CommandFrame> encode_command(const Command & command);
std::optional<Command> decode_command(const CommandFrame & frame);

struct Frame
{
  std::vector<std::uint8_t> payload;
  std::uint16_t crc = 0;
};

class StreamDecoder
{
public:
  void append(const std::uint8_t * data, std::size_t size);
  void append(const std::vector<std::uint8_t> & data);
  bool pop(Frame & frame);
  std::size_t buffered_size() const;
  void clear();

private:
  std::vector<std::uint8_t> buffer_;
};

}  // namespace hpm_crc_v1
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PROTOCOL_HPP_
