#include "rm_serial_driver/protocol.hpp"

#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace legacy = rm_serial_driver::legacy_v1;

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
