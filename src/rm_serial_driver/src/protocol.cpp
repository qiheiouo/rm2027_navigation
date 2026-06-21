#include "rm_serial_driver/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rm_serial_driver
{
namespace legacy_v1
{
namespace
{

void write_float_le(float value, std::uint8_t * output)
{
  std::uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "Unexpected float size");
  std::memcpy(&bits, &value, sizeof(bits));
  output[0] = static_cast<std::uint8_t>(bits & 0xffU);
  output[1] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  output[2] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  output[3] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

float read_float_le(const std::uint8_t * input)
{
  const std::uint32_t bits =
    static_cast<std::uint32_t>(input[0]) |
    (static_cast<std::uint32_t>(input[1]) << 8U) |
    (static_cast<std::uint32_t>(input[2]) << 16U) |
    (static_cast<std::uint32_t>(input[3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

std::optional<CommandFrame> encode_command(const Command & command)
{
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
    !std::isfinite(command.wz))
  {
    return std::nullopt;
  }

  CommandFrame frame{};
  frame[0] = kFrameHeader;
  frame[1] = static_cast<std::uint8_t>(kCommandPayloadSize);
  write_float_le(command.vx, frame.data() + 2);
  write_float_le(command.vy, frame.data() + 6);
  write_float_le(command.wz, frame.data() + 10);
  frame[14] = command.sequence;
  frame[15] = command.navigation_state;
  frame[16] = command.remake_command;
  frame[17] = command.bullet_command;
  frame[18] = command.allow_restart;
  return frame;
}

std::optional<Command> decode_command(const CommandFrame & frame)
{
  if (frame[0] != kFrameHeader || frame[1] != kCommandPayloadSize) {
    return std::nullopt;
  }

  Command command;
  command.vx = read_float_le(frame.data() + 2);
  command.vy = read_float_le(frame.data() + 6);
  command.wz = read_float_le(frame.data() + 10);
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
    !std::isfinite(command.wz))
  {
    return std::nullopt;
  }
  command.sequence = frame[14];
  command.navigation_state = frame[15];
  command.remake_command = frame[16];
  command.bullet_command = frame[17];
  command.allow_restart = frame[18];
  return command;
}

void StreamDecoder::append(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr || size == 0) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + size);
}

void StreamDecoder::append(const std::vector<std::uint8_t> & data)
{
  append(data.data(), data.size());
}

bool StreamDecoder::pop(Frame & frame)
{
  while (true) {
    const auto header = std::find(buffer_.begin(), buffer_.end(), kFrameHeader);
    if (header == buffer_.end()) {
      buffer_.clear();
      return false;
    }
    buffer_.erase(buffer_.begin(), header);

    if (buffer_.size() < 2) {
      return false;
    }

    const std::size_t payload_size = buffer_[1];
    if (payload_size > kMaxPayloadSize) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = 2 + payload_size;
    if (buffer_.size() < frame_size) {
      return false;
    }

    frame.payload.assign(buffer_.begin() + 2, buffer_.begin() + frame_size);
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
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

}  // namespace legacy_v1
}  // namespace rm_serial_driver
