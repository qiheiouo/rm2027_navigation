#include "competition_v2_codec.h"

#include <array>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

TEST(CompetitionV2Reference, MatchesCanonicalChassisGoldenVector)
{
  rmcv2_chassis_command_t command{};
  command.vx_mps = 1.0F;
  command.vy_mps = -2.0F;
  command.wz_rad_s = 0.5F;
  command.enabled = true;
  std::array<std::uint8_t, RMCV2_MAX_FRAME_SIZE> output{};
  std::size_t output_size = 0;
  ASSERT_EQ(
    rmcv2_encode_chassis_command(
      &command, 0x1234, output.data(), output.size(), &output_size),
    RMCV2_OK);
  const std::vector<std::uint8_t> expected{
    0x52, 0x4d, 0x02, 0x01, 0x0d, 0x00, 0x34, 0x12,
    0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0xc0,
    0x00, 0x00, 0x00, 0x3f, 0x01, 0xe6, 0xcb};
  EXPECT_EQ(std::vector<std::uint8_t>(output.begin(), output.begin() + output_size), expected);

  rmcv2_frame_t frame{};
  std::size_t consumed = 0;
  ASSERT_EQ(rmcv2_decode_frame(output.data(), output_size, &frame, &consumed), RMCV2_OK);
  EXPECT_EQ(consumed, output_size);
  rmcv2_chassis_command_t decoded{};
  ASSERT_EQ(rmcv2_decode_chassis_command(&frame, &decoded), RMCV2_OK);
  EXPECT_FLOAT_EQ(decoded.vx_mps, command.vx_mps);
  EXPECT_FLOAT_EQ(decoded.vy_mps, command.vy_mps);
  EXPECT_FLOAT_EQ(decoded.wz_rad_s, command.wz_rad_s);
  EXPECT_TRUE(decoded.enabled);
}

TEST(CompetitionV2Reference, SupportsAllPosturesAndSequenceWrap)
{
  for (std::uint8_t posture = 0; posture <= 5; ++posture) {
    EXPECT_TRUE(rmcv2_posture_is_valid(posture));
  }
  EXPECT_FALSE(rmcv2_posture_is_valid(6));
  EXPECT_TRUE(rmcv2_sequence_is_newer(0, 0xffff));
  EXPECT_FALSE(rmcv2_sequence_is_newer(0xffff, 0));
  EXPECT_TRUE(rmcv2_command_id_is_newer(0U, 0xffffffffU));
  EXPECT_FALSE(rmcv2_command_id_is_newer(0xffffffffU, 0U));
}

TEST(CompetitionV2Reference, RoundTripsEveryMessageType)
{
  std::array<std::uint8_t, RMCV2_MAX_FRAME_SIZE> bytes{};
  std::size_t size = 0;
  std::size_t consumed = 0;
  rmcv2_frame_t frame{};

  rmcv2_posture_request_t posture_request{};
  posture_request.command_id = 0xffffffffU;
  posture_request.requester_boot_id = 0x12345678U;
  posture_request.requested_posture = RMCV2_POSTURE_ENHANCED_MOVE;
  posture_request.valid = true;
  ASSERT_EQ(
    rmcv2_encode_posture_request(
      &posture_request, 1, bytes.data(), bytes.size(), &size),
    RMCV2_OK);
  ASSERT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_OK);
  rmcv2_posture_request_t decoded_request{};
  ASSERT_EQ(rmcv2_decode_posture_request(&frame, &decoded_request), RMCV2_OK);
  EXPECT_EQ(decoded_request.command_id, posture_request.command_id);
  EXPECT_EQ(decoded_request.requested_posture, posture_request.requested_posture);

  rmcv2_posture_state_t posture_state{};
  posture_state.ack_command_id = posture_request.command_id;
  posture_state.ack_requester_boot_id = posture_request.requester_boot_id;
  posture_state.posture_target = RMCV2_POSTURE_ENHANCED_MOVE;
  posture_state.posture_actual = RMCV2_POSTURE_MOVE;
  posture_state.valid = true;
  posture_state.online = true;
  posture_state.transitioning = true;
  posture_state.fault = true;
  posture_state.fault_code = 7;
  ASSERT_EQ(
    rmcv2_encode_posture_state(&posture_state, 2, bytes.data(), bytes.size(), &size),
    RMCV2_OK);
  ASSERT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_OK);
  rmcv2_posture_state_t decoded_state{};
  ASSERT_EQ(rmcv2_decode_posture_state(&frame, &decoded_state), RMCV2_OK);
  EXPECT_TRUE(decoded_state.transitioning);
  EXPECT_TRUE(decoded_state.fault);
  EXPECT_EQ(decoded_state.fault_code, 7);

  rmcv2_gimbal_state_t gimbal{};
  gimbal.relative_yaw_rad = -3.14159265358979323846F;
  gimbal.yaw_rate_rad_s = 1.5F;
  gimbal.sample_sequence = 0xffffffffU;
  gimbal.mcu_time_ms = 1234;
  gimbal.valid = true;
  gimbal.online = true;
  ASSERT_EQ(
    rmcv2_encode_gimbal_state(&gimbal, 3, bytes.data(), bytes.size(), &size),
    RMCV2_OK);
  ASSERT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_OK);
  rmcv2_gimbal_state_t decoded_gimbal{};
  ASSERT_EQ(rmcv2_decode_gimbal_state(&frame, &decoded_gimbal), RMCV2_OK);
  EXPECT_FLOAT_EQ(decoded_gimbal.relative_yaw_rad, gimbal.relative_yaw_rad);

  rmcv2_chassis_heading_state_t heading{};
  heading.yaw_rad = 2.75F;
  heading.yaw_rate_rad_s = -1.25F;
  heading.sample_sequence = 0x10203040U;
  heading.mcu_time_ms = 4321U;
  heading.reset_counter = 17U;
  heading.valid = true;
  heading.online = true;
  ASSERT_EQ(
    rmcv2_encode_chassis_heading_state(&heading, 4, bytes.data(), bytes.size(), &size),
    RMCV2_OK);
  ASSERT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_OK);
  rmcv2_chassis_heading_state_t decoded_heading{};
  ASSERT_EQ(
    rmcv2_decode_chassis_heading_state(&frame, &decoded_heading), RMCV2_OK);
  EXPECT_FLOAT_EQ(decoded_heading.yaw_rad, heading.yaw_rad);
  EXPECT_FLOAT_EQ(decoded_heading.yaw_rate_rad_s, heading.yaw_rate_rad_s);
  EXPECT_EQ(decoded_heading.reset_counter, heading.reset_counter);

  rmcv2_referee_state_t referee{};
  referee.game_progress = 4;
  referee.stage_remain_time = 299;
  referee.robot_id = 107;
  referee.current_hp = 400;
  referee.red_outpost_hp = 1200;
  referee.blue_outpost_hp = 1100;
  referee.projectile_allowance_17mm = 88;
  referee.remaining_gold_coin = 25;
  referee.diagnostic_flags = 0x55aa;
  referee.valid = true;
  ASSERT_EQ(
    rmcv2_encode_referee_state(&referee, 5, bytes.data(), bytes.size(), &size),
    RMCV2_OK);
  ASSERT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_OK);
  rmcv2_referee_state_t decoded_referee{};
  ASSERT_EQ(rmcv2_decode_referee_state(&frame, &decoded_referee), RMCV2_OK);
  EXPECT_EQ(decoded_referee.diagnostic_flags, referee.diagnostic_flags);

  rmcv2_operator_target_t target{};
  target.command_id = 9;
  target.coordinate_frame = RMCV2_FRAME_MAP;
  target.alliance = RMCV2_ALLIANCE_BLUE;
  target.x_m = 0.0F;
  target.y_m = 0.0F;
  target.yaw_rad = 0.5F;
  target.sample_time_ms = 5678;
  target.ttl_ms = 1000;
  target.valid = true;
  target.has_yaw = true;
  ASSERT_EQ(
    rmcv2_encode_operator_target(&target, 6, bytes.data(), bytes.size(), &size),
    RMCV2_OK);
  ASSERT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_OK);
  rmcv2_operator_target_t decoded_target{};
  ASSERT_EQ(rmcv2_decode_operator_target(&frame, &decoded_target), RMCV2_OK);
  EXPECT_FLOAT_EQ(decoded_target.x_m, 0.0F);
  EXPECT_FLOAT_EQ(decoded_target.y_m, 0.0F);

  rmcv2_heartbeat_t heartbeat{};
  heartbeat.uptime_ms = 99;
  heartbeat.boot_id = 0xabcdef01U;
  heartbeat.capabilities = RMCV2_CAP_POSTURE | RMCV2_CAP_GIMBAL_STATE;
  heartbeat.ready = true;
  ASSERT_EQ(
    rmcv2_encode_heartbeat(&heartbeat, 7, bytes.data(), bytes.size(), &size),
    RMCV2_OK);
  ASSERT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_OK);
  rmcv2_heartbeat_t decoded_heartbeat{};
  ASSERT_EQ(rmcv2_decode_heartbeat(&frame, &decoded_heartbeat), RMCV2_OK);
  EXPECT_EQ(decoded_heartbeat.boot_id, heartbeat.boot_id);
}

TEST(CompetitionV2Reference, ReportsFragmentationAndConsumesOneByteOnCorruption)
{
  rmcv2_heartbeat_t heartbeat{};
  std::array<std::uint8_t, RMCV2_MAX_FRAME_SIZE> bytes{};
  std::size_t size = 0;
  ASSERT_EQ(
    rmcv2_encode_heartbeat(&heartbeat, 7, bytes.data(), bytes.size(), &size),
    RMCV2_OK);

  rmcv2_frame_t frame{};
  std::size_t consumed = 99;
  EXPECT_EQ(
    rmcv2_decode_frame(bytes.data(), RMCV2_HEADER_SIZE - 1, &frame, &consumed),
    RMCV2_NEED_MORE);
  EXPECT_EQ(consumed, 0U);

  bytes[size - 1] ^= 0x80U;
  EXPECT_EQ(rmcv2_decode_frame(bytes.data(), size, &frame, &consumed), RMCV2_INVALID_FRAME);
  EXPECT_EQ(consumed, 1U);
}
