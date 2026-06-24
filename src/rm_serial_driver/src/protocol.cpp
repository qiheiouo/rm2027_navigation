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

namespace hpm_crc_v1
{

std::uint16_t crc16_modbus(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr && size != 0U) {
    return 0U;
  }

  std::uint16_t crc = 0xffffU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = static_cast<std::uint16_t>((crc >> 1U) ^ 0xa001U);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1U);
      }
    }
  }
  return crc;
}

std::optional<CommandFrame> encode_command(const Command & command)
{
  const auto legacy_frame = legacy_v1::encode_command(command);
  if (!legacy_frame.has_value()) {
    return std::nullopt;
  }

  CommandFrame frame{};
  std::copy(legacy_frame->begin(), legacy_frame->end(), frame.begin());
  const auto crc = crc16_modbus(frame.data() + 2, kCommandPayloadSize);
  frame[2 + kCommandPayloadSize] = static_cast<std::uint8_t>(crc & 0xffU);
  frame[3 + kCommandPayloadSize] = static_cast<std::uint8_t>((crc >> 8U) & 0xffU);
  return frame;
}

std::optional<Command> decode_command(const CommandFrame & frame)
{
  if (frame[0] != legacy_v1::kFrameHeader || frame[1] != kCommandPayloadSize) {
    return std::nullopt;
  }

  const auto expected_crc = crc16_modbus(frame.data() + 2, kCommandPayloadSize);
  const auto received_crc = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame[2 + kCommandPayloadSize]) |
    (static_cast<std::uint16_t>(frame[3 + kCommandPayloadSize]) << 8U));
  if (expected_crc != received_crc) {
    return std::nullopt;
  }

  legacy_v1::CommandFrame legacy_frame{};
  std::copy_n(frame.begin(), legacy_frame.size(), legacy_frame.begin());
  return legacy_v1::decode_command(legacy_frame);
}

void StreamDecoder::append(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr || size == 0U) {
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
    const auto header = std::find(
      buffer_.begin(), buffer_.end(), legacy_v1::kFrameHeader);
    if (header == buffer_.end()) {
      buffer_.clear();
      return false;
    }
    buffer_.erase(buffer_.begin(), header);

    if (buffer_.size() < 2U) {
      return false;
    }

    const std::size_t payload_size = buffer_[1];
    if (payload_size > kMaxPayloadSize) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = 2U + payload_size + kCrcSize;
    if (buffer_.size() < frame_size) {
      return false;
    }

    const auto expected_crc = crc16_modbus(buffer_.data() + 2, payload_size);
    const auto received_crc = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(buffer_[2 + payload_size]) |
      (static_cast<std::uint16_t>(buffer_[3 + payload_size]) << 8U));
    if (expected_crc != received_crc) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    frame.payload.assign(buffer_.begin() + 2, buffer_.begin() + 2 + payload_size);
    frame.crc = received_crc;
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

}  // namespace hpm_crc_v1
}  // namespace rm_serial_driver
