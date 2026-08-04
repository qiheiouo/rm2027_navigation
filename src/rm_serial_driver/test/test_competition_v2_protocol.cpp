#include "rm_serial_driver/competition_v2_protocol.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace v2 = rm_serial_driver::competition_v2;

namespace
{

v2::Frame decodeSingle(const std::vector<std::uint8_t> & bytes)
{
  v2::StreamDecoder decoder;
  decoder.append(bytes);
  v2::Frame frame;
  EXPECT_TRUE(decoder.pop(frame));
  EXPECT_EQ(decoder.buffered_size(), 0U);
  return frame;
}

}  // namespace

TEST(CompetitionV2Protocol, ChassisCommandMatchesGoldenBytes)
{
  v2::ChassisCommand command;
  command.vx_mps = 1.0F;
  command.vy_mps = -2.0F;
  command.wz_rad_s = 0.5F;
  command.enabled = true;
  const auto encoded = v2::encode_message(v2::MessageType::ChassisCommand, 0x1234, command);
  ASSERT_TRUE(encoded.has_value());
  const std::vector<std::uint8_t> expected{
    0x52, 0x4d, 0x02, 0x01, 0x0d, 0x00, 0x34, 0x12,
    0x00, 0x00, 0x80, 0x3f,
    0x00, 0x00, 0x00, 0xc0,
    0x00, 0x00, 0x00, 0x3f,
    0x01, 0xe6, 0xcb};
  EXPECT_EQ(*encoded, expected);

  const auto decoded = v2::decode_chassis_command(decodeSingle(*encoded));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->vx_mps, command.vx_mps);
  EXPECT_FLOAT_EQ(decoded->vy_mps, command.vy_mps);
  EXPECT_FLOAT_EQ(decoded->wz_rad_s, command.wz_rad_s);
  EXPECT_TRUE(decoded->enabled);
}

TEST(CompetitionV2Protocol, RoundTripsAllSixPosturesAndSessionIdentity)
{
  for (std::uint8_t value = 0; value <= 5; ++value) {
    v2::PostureRequest request;
    request.command_id = 0xffffffffU - value;
    request.requester_boot_id = 0x10203040U;
    request.requested_posture = static_cast<v2::Posture>(value);
    request.valid = true;
    request.request_flags = 0x80U;
    const auto bytes = v2::encode_message(v2::MessageType::PostureRequest, value, request);
    ASSERT_TRUE(bytes.has_value());
    const auto decoded = v2::decode_posture_request(decodeSingle(*bytes));
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->command_id, request.command_id);
    EXPECT_EQ(decoded->requester_boot_id, request.requester_boot_id);
    EXPECT_EQ(decoded->requested_posture, request.requested_posture);
    EXPECT_TRUE(decoded->valid);
    EXPECT_EQ(decoded->request_flags, request.request_flags);
  }

  v2::PostureRequest invalid;
  invalid.requested_posture = static_cast<v2::Posture>(6);
  EXPECT_FALSE(v2::encode_payload(invalid).has_value());
}

TEST(CompetitionV2Protocol, RoundTripsPostureState)
{
  v2::PostureState state;
  state.ack_command_id = 42;
  state.ack_requester_boot_id = 0xaabbccddU;
  state.posture_target = v2::Posture::EnhancedDefense;
  state.posture_actual = v2::Posture::Defense;
  state.valid = true;
  state.online = true;
  state.transitioning = true;
  state.fault = true;
  state.fault_code = 0x1234;
  const auto bytes = v2::encode_message(v2::MessageType::PostureState, 3, state);
  ASSERT_TRUE(bytes.has_value());
  const auto decoded = v2::decode_posture_state(decodeSingle(*bytes));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->ack_command_id, state.ack_command_id);
  EXPECT_EQ(decoded->ack_requester_boot_id, state.ack_requester_boot_id);
  EXPECT_EQ(decoded->posture_target, state.posture_target);
  EXPECT_EQ(decoded->posture_actual, state.posture_actual);
  EXPECT_TRUE(decoded->transitioning);
  EXPECT_TRUE(decoded->fault);
  EXPECT_EQ(decoded->fault_code, state.fault_code);
}

TEST(CompetitionV2Protocol, GimbalYawBoundariesAndNonFiniteValues)
{
  for (const float yaw : {-3.14159265358979323846F, -0.0F, 3.1415924F}) {
    v2::GimbalState state;
    state.relative_yaw_rad = yaw;
    state.yaw_rate_rad_s = -6.0F;
    state.sample_sequence = 0xffffffffU;
    state.mcu_time_ms = 1234;
    state.valid = true;
    state.online = true;
    const auto bytes = v2::encode_message(v2::MessageType::GimbalState, 9, state);
    ASSERT_TRUE(bytes.has_value());
    const auto decoded = v2::decode_gimbal_state(decodeSingle(*bytes));
    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->relative_yaw_rad, yaw);
    EXPECT_FLOAT_EQ(decoded->yaw_rate_rad_s, state.yaw_rate_rad_s);
  }

  v2::GimbalState invalid;
  invalid.relative_yaw_rad = 3.14159265358979323846F;
  EXPECT_FALSE(v2::encode_payload(invalid).has_value());
  invalid.relative_yaw_rad = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(v2::encode_payload(invalid).has_value());
  invalid.relative_yaw_rad = 0.0F;
  invalid.yaw_rate_rad_s = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(v2::encode_payload(invalid).has_value());
}

TEST(CompetitionV2Protocol, OperatorTargetTreatsOriginAsValid)
{
  v2::OperatorNavigationTarget target;
  target.command_id = 17;
  target.coordinate_frame = v2::CoordinateFrame::Map;
  target.alliance = v2::Alliance::Red;
  target.x_m = 0.0F;
  target.y_m = 0.0F;
  target.yaw_rad = -1.0F;
  target.sample_time_ms = 500;
  target.ttl_ms = 1000;
  target.valid = true;
  target.has_yaw = true;
  const auto bytes = v2::encode_message(
    v2::MessageType::OperatorNavigationTarget, 10, target);
  ASSERT_TRUE(bytes.has_value());
  const auto decoded = v2::decode_operator_navigation_target(decodeSingle(*bytes));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->x_m, 0.0F);
  EXPECT_FLOAT_EQ(decoded->y_m, 0.0F);
  EXPECT_TRUE(decoded->valid);
  EXPECT_TRUE(decoded->has_yaw);

  target.coordinate_frame = static_cast<v2::CoordinateFrame>(99);
  EXPECT_FALSE(v2::encode_payload(target).has_value());
}

TEST(CompetitionV2Protocol, RoundTripsRefereeAndHeartbeat)
{
  v2::RefereeState referee;
  referee.game_progress = 4;
  referee.stage_remain_time = 299;
  referee.robot_id = 107;
  referee.current_hp = 400;
  referee.red_outpost_hp = 1200;
  referee.blue_outpost_hp = 1100;
  referee.projectile_allowance_17mm = 88;
  referee.remaining_gold_coin = 25;
  referee.diagnostic_flags = 0xa55a;
  referee.valid = true;
  const auto referee_bytes = v2::encode_message(v2::MessageType::RefereeState, 1, referee);
  ASSERT_TRUE(referee_bytes.has_value());
  const auto decoded_referee = v2::decode_referee_state(decodeSingle(*referee_bytes));
  ASSERT_TRUE(decoded_referee.has_value());
  EXPECT_EQ(decoded_referee->robot_id, referee.robot_id);
  EXPECT_EQ(decoded_referee->diagnostic_flags, referee.diagnostic_flags);

  v2::Heartbeat heartbeat;
  heartbeat.uptime_ms = 123456;
  heartbeat.boot_id = 0x11223344;
  heartbeat.capabilities = v2::CapabilityChassisCommand | v2::CapabilityPosture;
  heartbeat.ready = true;
  const auto heartbeat_bytes = v2::encode_message(v2::MessageType::Heartbeat, 2, heartbeat);
  ASSERT_TRUE(heartbeat_bytes.has_value());
  const auto decoded_heartbeat = v2::decode_heartbeat(decodeSingle(*heartbeat_bytes));
  ASSERT_TRUE(decoded_heartbeat.has_value());
  EXPECT_EQ(decoded_heartbeat->boot_id, heartbeat.boot_id);
  EXPECT_EQ(decoded_heartbeat->capabilities, heartbeat.capabilities);
  EXPECT_TRUE(decoded_heartbeat->ready);
}

TEST(CompetitionV2Protocol, RejectsCorruptionAndResynchronizesAfterNoise)
{
  v2::Heartbeat heartbeat;
  heartbeat.boot_id = 7;
  const auto valid = v2::encode_message(v2::MessageType::Heartbeat, 5, heartbeat);
  ASSERT_TRUE(valid.has_value());
  auto corrupted = *valid;
  corrupted[10] ^= 0x80U;

  std::vector<std::uint8_t> stream{0x00, 0x52, 0x00, 0xff};
  stream.insert(stream.end(), corrupted.begin(), corrupted.end());
  stream.insert(stream.end(), valid->begin(), valid->end());
  v2::StreamDecoder decoder;
  decoder.append(stream);
  v2::Frame frame;
  ASSERT_TRUE(decoder.pop(frame));
  EXPECT_EQ(frame.sequence, 5);
  EXPECT_GT(decoder.crc_error_count(), 0U);
  EXPECT_GT(decoder.framing_error_count(), 0U);
}

TEST(CompetitionV2Protocol, HandlesFragmentationAndConcatenatedFrames)
{
  v2::Heartbeat first;
  first.boot_id = 1;
  v2::Heartbeat second;
  second.boot_id = 2;
  const auto first_bytes = v2::encode_message(v2::MessageType::Heartbeat, 10, first);
  const auto second_bytes = v2::encode_message(v2::MessageType::Heartbeat, 11, second);
  ASSERT_TRUE(first_bytes.has_value());
  ASSERT_TRUE(second_bytes.has_value());

  v2::StreamDecoder decoder;
  decoder.append(first_bytes->data(), 3);
  v2::Frame frame;
  EXPECT_FALSE(decoder.pop(frame));
  decoder.append(first_bytes->data() + 3, first_bytes->size() - 3);
  decoder.append(*second_bytes);
  ASSERT_TRUE(decoder.pop(frame));
  EXPECT_EQ(frame.sequence, 10);
  ASSERT_TRUE(decoder.pop(frame));
  EXPECT_EQ(frame.sequence, 11);
  EXPECT_FALSE(decoder.pop(frame));
}

TEST(CompetitionV2Protocol, RejectsInvalidVersionAndLength)
{
  std::vector<std::uint8_t> invalid_version{
    v2::kMagic0, v2::kMagic1, 3, 1, 0, 0, 0, 0, 0, 0};
  std::vector<std::uint8_t> invalid_length{
    v2::kMagic0, v2::kMagic1, v2::kProtocolVersion, 1, 0xff, 0xff, 0, 0};
  v2::Heartbeat heartbeat;
  const auto valid = v2::encode_message(v2::MessageType::Heartbeat, 4, heartbeat);
  ASSERT_TRUE(valid.has_value());
  invalid_version.insert(invalid_version.end(), invalid_length.begin(), invalid_length.end());
  invalid_version.insert(invalid_version.end(), valid->begin(), valid->end());
  v2::StreamDecoder decoder;
  decoder.append(invalid_version);
  v2::Frame frame;
  ASSERT_TRUE(decoder.pop(frame));
  EXPECT_EQ(frame.sequence, 4);
  EXPECT_GT(decoder.framing_error_count(), 0U);
}

TEST(CompetitionV2Protocol, PreservesUnknownMessageForForwardCompatibility)
{
  v2::Frame unknown;
  unknown.message_type = 0x55;
  unknown.sequence = 8;
  unknown.payload = {1, 2, 3};
  const auto encoded = v2::encode_frame(unknown);
  ASSERT_TRUE(encoded.has_value());
  const auto decoded = decodeSingle(*encoded);
  EXPECT_EQ(decoded.message_type, 0x55);
  EXPECT_FALSE(v2::is_known_message_type(decoded.message_type));
  EXPECT_FALSE(v2::decode_heartbeat(decoded).has_value());
}

TEST(CompetitionV2Protocol, SequenceComparisonHandlesWrapAndDuplicates)
{
  EXPECT_TRUE(v2::is_newer_sequence(1, 0));
  EXPECT_FALSE(v2::is_newer_sequence(7, 7));
  EXPECT_TRUE(v2::is_newer_sequence(0, 0xffff));
  EXPECT_FALSE(v2::is_newer_sequence(0xffff, 0));
  EXPECT_TRUE(v2::is_newer_command_id(0U, 0xffffffffU));
  EXPECT_FALSE(v2::is_newer_command_id(0xffffffffU, 0U));
  EXPECT_FALSE(v2::is_newer_command_id(42U, 42U));
}

TEST(CompetitionV2Protocol, RejectsNonFiniteChassisVelocity)
{
  v2::ChassisCommand command;
  command.vx_mps = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(v2::encode_payload(command).has_value());
}

TEST(CompetitionV2Protocol, BoundsStreamBufferUnderNoise)
{
  v2::StreamDecoder decoder;
  std::vector<std::uint8_t> noise(v2::kMaxStreamBufferSize + 100U, 0x7f);
  decoder.append(noise);
  EXPECT_LE(decoder.buffered_size(), v2::kMaxStreamBufferSize);
  EXPECT_GE(decoder.framing_error_count(), 100U);
}
