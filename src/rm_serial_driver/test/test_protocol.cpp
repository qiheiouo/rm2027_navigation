#include "rm_serial_driver/protocol.hpp"

#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace legacy = rm_serial_driver::legacy_v1;
namespace hpm = rm_serial_driver::hpm_crc_v1;
namespace referee = rm_serial_driver::hpm_referee_v1;

namespace
{

void append_u16_le(std::vector<std::uint8_t> & bytes, std::uint16_t value)
{
  bytes.push_back(static_cast<std::uint8_t>(value & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
}

void append_float_le(std::vector<std::uint8_t> & bytes, float value)
{
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  bytes.push_back(static_cast<std::uint8_t>(bits & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((bits >> 8U) & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((bits >> 16U) & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((bits >> 24U) & 0xffU));
}

}  // namespace

TEST(LegacyProtocol, EncodesKnownCommandLayout)
{
  legacy::Command command;
  command.vx = 1.0F;
  command.vy = -2.0F;
  command.wz = 0.5F;
  command.sequence = 0x12;
  command.navigation_state = 0x34;
  command.remake_command = 0x56;
  command.bullet_command = 0x78;
  command.allow_restart = 0x9a;

  const auto encoded = legacy::encode_command(command);
  ASSERT_TRUE(encoded.has_value());
  const legacy::CommandFrame expected = {
    0x3e, 0x11,
    0x00, 0x00, 0x80, 0x3f,
    0x00, 0x00, 0x00, 0xc0,
    0x00, 0x00, 0x00, 0x3f,
    0x12, 0x34, 0x56, 0x78, 0x9a};
  EXPECT_EQ(*encoded, expected);
}

TEST(LegacyProtocol, RoundTripsCommand)
{
  legacy::Command command;
  command.vx = -0.25F;
  command.vy = 0.75F;
  command.wz = -1.5F;
  command.sequence = 7;
  command.navigation_state = 8;
  command.remake_command = 9;
  command.bullet_command = 10;
  command.allow_restart = 11;

  const auto encoded = legacy::encode_command(command);
  ASSERT_TRUE(encoded.has_value());
  const auto decoded = legacy::decode_command(*encoded);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->vx, command.vx);
  EXPECT_FLOAT_EQ(decoded->vy, command.vy);
  EXPECT_FLOAT_EQ(decoded->wz, command.wz);
  EXPECT_EQ(decoded->sequence, command.sequence);
  EXPECT_EQ(decoded->navigation_state, command.navigation_state);
  EXPECT_EQ(decoded->remake_command, command.remake_command);
  EXPECT_EQ(decoded->bullet_command, command.bullet_command);
  EXPECT_EQ(decoded->allow_restart, command.allow_restart);
}

TEST(LegacyProtocol, RejectsNonFiniteVelocity)
{
  legacy::Command command;
  command.vx = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(legacy::encode_command(command).has_value());
}

TEST(LegacyProtocol, ReassemblesFragmentedFrameAfterNoise)
{
  legacy::Command command;
  command.vx = 0.1F;
  const auto encoded = legacy::encode_command(command);
  ASSERT_TRUE(encoded.has_value());

  legacy::StreamDecoder decoder;
  decoder.append(std::vector<std::uint8_t>{0x00, 0x7f, legacy::kFrameHeader});
  legacy::Frame frame;
  EXPECT_FALSE(decoder.pop(frame));
  decoder.append(encoded->data() + 1, encoded->size() - 1);
  ASSERT_TRUE(decoder.pop(frame));
  ASSERT_EQ(frame.payload.size(), legacy::kCommandPayloadSize);
  EXPECT_EQ(frame.payload[12], command.sequence);
  EXPECT_EQ(decoder.buffered_size(), 0U);
}

TEST(LegacyProtocol, ResynchronizesAfterInvalidLength)
{
  legacy::Command command;
  command.sequence = 0x42;
  const auto encoded = legacy::encode_command(command);
  ASSERT_TRUE(encoded.has_value());

  std::vector<std::uint8_t> bytes{legacy::kFrameHeader, 0xff, 0x00};
  bytes.insert(bytes.end(), encoded->begin(), encoded->end());

  legacy::StreamDecoder decoder;
  decoder.append(bytes);
  legacy::Frame frame;
  ASSERT_TRUE(decoder.pop(frame));
  ASSERT_EQ(frame.payload.size(), legacy::kCommandPayloadSize);
  EXPECT_EQ(frame.payload[12], 0x42);
}

TEST(HpmCrcProtocol, MatchesStandardModbusCheckVector)
{
  const std::vector<std::uint8_t> bytes{
    '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(hpm::crc16_modbus(bytes.data(), bytes.size()), 0x4b37U);
}

TEST(HpmCrcProtocol, EncodesKnownCommandAndPayloadCrc)
{
  hpm::Command command;
  command.vx = 1.0F;
  command.vy = -2.0F;
  command.wz = 0.5F;
  command.sequence = 0x12;
  command.navigation_state = 0x34;
  command.remake_command = 0x56;
  command.bullet_command = 0x78;
  command.allow_restart = 0x9a;

  const auto encoded = hpm::encode_command(command);
  ASSERT_TRUE(encoded.has_value());
  const hpm::CommandFrame expected = {
    0x3e, 0x11,
    0x00, 0x00, 0x80, 0x3f,
    0x00, 0x00, 0x00, 0xc0,
    0x00, 0x00, 0x00, 0x3f,
    0x12, 0x34, 0x56, 0x78, 0x9a,
    0x3b, 0x54};
  EXPECT_EQ(*encoded, expected);
}

TEST(HpmCrcProtocol, RoundTripsAndRejectsCorruption)
{
  hpm::Command command;
  command.vx = -0.25F;
  command.vy = 0.75F;
  command.wz = -1.5F;
  command.sequence = 7;

  const auto encoded = hpm::encode_command(command);
  ASSERT_TRUE(encoded.has_value());
  const auto decoded = hpm::decode_command(*encoded);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->vx, command.vx);
  EXPECT_FLOAT_EQ(decoded->vy, command.vy);
  EXPECT_FLOAT_EQ(decoded->wz, command.wz);
  EXPECT_EQ(decoded->sequence, command.sequence);

  auto corrupted = *encoded;
  corrupted[6] ^= 0x01U;
  EXPECT_FALSE(hpm::decode_command(corrupted).has_value());
}

TEST(HpmCrcProtocol, ReassemblesFragmentedFrameAfterNoise)
{
  hpm::Command command;
  command.vx = 0.1F;
  command.sequence = 0x23;
  const auto encoded = hpm::encode_command(command);
  ASSERT_TRUE(encoded.has_value());

  hpm::StreamDecoder decoder;
  decoder.append(std::vector<std::uint8_t>{0x00, 0x7f, legacy::kFrameHeader});
  hpm::Frame frame;
  EXPECT_FALSE(decoder.pop(frame));
  decoder.append(encoded->data() + 1, encoded->size() - 1);
  ASSERT_TRUE(decoder.pop(frame));
  ASSERT_EQ(frame.payload.size(), hpm::kCommandPayloadSize);
  EXPECT_EQ(frame.payload[12], 0x23);
  EXPECT_EQ(decoder.buffered_size(), 0U);
}

TEST(HpmCrcProtocol, ResynchronizesAfterCorruptedFrame)
{
  hpm::Command command;
  command.sequence = 0x42;
  const auto encoded = hpm::encode_command(command);
  ASSERT_TRUE(encoded.has_value());

  auto corrupted = *encoded;
  corrupted[10] ^= 0x80U;
  std::vector<std::uint8_t> bytes(corrupted.begin(), corrupted.end());
  bytes.insert(bytes.end(), encoded->begin(), encoded->end());

  hpm::StreamDecoder decoder;
  decoder.append(bytes);
  hpm::Frame frame;
  ASSERT_TRUE(decoder.pop(frame));
  ASSERT_EQ(frame.payload.size(), hpm::kCommandPayloadSize);
  EXPECT_EQ(frame.payload[12], 0x42);
  EXPECT_EQ(decoder.buffered_size(), 0U);
}

TEST(HpmRefereeProtocol, DecodesCurrentFlashedFeedbackLayout)
{
  std::vector<std::uint8_t> payload;
  append_float_le(payload, 1.25F);
  append_float_le(payload, 2.5F);
  append_float_le(payload, -3.75F);
  payload.push_back(4);
  append_u16_le(payload, 299);
  append_u16_le(payload, 1200);
  append_u16_le(payload, 1100);
  payload.push_back(107);
  append_u16_le(payload, 400);
  append_u16_le(payload, 88);
  append_u16_le(payload, 25);
  payload.insert(payload.end(), {1, 2, 3, 4});
  payload.push_back(5);
  append_float_le(payload, 6.5F);
  payload.push_back(1);
  payload.push_back(0);
  append_float_le(payload, 7.5F);
  ASSERT_EQ(payload.size(), referee::kFeedbackPayloadSize);

  std::vector<std::uint8_t> bytes{legacy::kFrameHeader, referee::kFeedbackPayloadSize};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  const auto crc = hpm::crc16_modbus(payload.data(), payload.size());
  bytes.push_back(static_cast<std::uint8_t>(crc & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((crc >> 8U) & 0xffU));
  ASSERT_EQ(bytes.size(), referee::kFeedbackFrameSize);

  hpm::StreamDecoder stream;
  stream.append(bytes.data(), 7);
  hpm::Frame frame;
  EXPECT_FALSE(stream.pop(frame));
  stream.append(bytes.data() + 7, bytes.size() - 7);
  ASSERT_TRUE(stream.pop(frame));
  const auto feedback = referee::decode_feedback(frame);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_FLOAT_EQ(feedback->yaw, 1.25F);
  EXPECT_FLOAT_EQ(feedback->target_position_x, 2.5F);
  EXPECT_FLOAT_EQ(feedback->target_position_y, -3.75F);
  EXPECT_EQ(feedback->game_progress, 4);
  EXPECT_EQ(feedback->stage_remain_time, 299);
  EXPECT_EQ(feedback->red_outpost_hp, 1200);
  EXPECT_EQ(feedback->blue_outpost_hp, 1100);
  EXPECT_EQ(feedback->robot_id, 107);
  EXPECT_EQ(feedback->current_hp, 400);
  EXPECT_EQ(feedback->projectile_allowance_17mm, 88);
  EXPECT_EQ(feedback->remaining_gold_coin, 25);
  EXPECT_EQ(feedback->sentry_info, (std::array<std::uint8_t, 4>{1, 2, 3, 4}));
  EXPECT_EQ(feedback->keyboard_command, 5);
  EXPECT_FLOAT_EQ(feedback->target_distance, 6.5F);
  EXPECT_EQ(feedback->life, 1);
  EXPECT_EQ(feedback->chassis_detect_error, 0);
  EXPECT_FLOAT_EQ(feedback->redundancy, 7.5F);
}

TEST(HpmRefereeProtocol, RejectsWrongPayloadSize)
{
  hpm::Frame frame;
  frame.payload.resize(referee::kFeedbackPayloadSize - 1);
  EXPECT_FALSE(referee::decode_feedback(frame).has_value());
}
