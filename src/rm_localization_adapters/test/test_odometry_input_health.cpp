#include <cstdint>

#include "gtest/gtest.h"
#include "rm_localization_adapters/odometry_input_health.hpp"

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

rm_localization_adapters::OdometryInputHealthConfig enabled_config()
{
  rm_localization_adapters::OdometryInputHealthConfig config;
  config.enabled = true;
  config.max_message_age_sec = 0.2;
  config.max_future_offset_sec = 0.05;
  config.recovery_fresh_samples = 3U;
  config.recovery_max_translation_m = 0.75;
  return config;
}

TEST(OdometryInputHealthGate, HealthyStreamUpdatesAnchor)
{
  rm_localization_adapters::OdometryInputHealthGate gate(enabled_config());
  EXPECT_EQ(
    gate.evaluate(10 * kSecond, 10 * kSecond - 20000000LL, 1.0, 2.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kAccept);
  EXPECT_FALSE(gate.latched());
  EXPECT_TRUE(gate.has_anchor());
}

TEST(OdometryInputHealthGate, StaleBacklogLatchesAndNeedsFreshConsensus)
{
  rm_localization_adapters::OdometryInputHealthGate gate(enabled_config());
  ASSERT_EQ(
    gate.evaluate(10 * kSecond, 10 * kSecond, 1.0, 2.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kAccept);
  EXPECT_EQ(
    gate.evaluate(14 * kSecond, 11 * kSecond, 1.0, 2.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kRejectStale);
  EXPECT_TRUE(gate.latched());

  EXPECT_EQ(
    gate.evaluate(14 * kSecond, 14 * kSecond, 1.02, 2.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kRecovering);
  EXPECT_EQ(
    gate.evaluate(14 * kSecond + 20000000LL, 14 * kSecond + 20000000LL, 1.03, 2.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kRecovering);
  EXPECT_EQ(
    gate.evaluate(14 * kSecond + 40000000LL, 14 * kSecond + 40000000LL, 1.04, 2.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kAccept);
  EXPECT_FALSE(gate.latched());
}

TEST(OdometryInputHealthGate, RejectsFreshButDisplacedPostBacklogPose)
{
  rm_localization_adapters::OdometryInputHealthGate gate(enabled_config());
  ASSERT_EQ(
    gate.evaluate(10 * kSecond, 10 * kSecond, 1.0, 2.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kAccept);
  gate.latch_silence();
  EXPECT_EQ(
    gate.evaluate(14 * kSecond, 14 * kSecond, 1.0, 12.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kRejectAnchor);
  EXPECT_TRUE(gate.latched());
  EXPECT_EQ(gate.recovery_count(), 0U);
}

TEST(OdometryInputHealthGate, RejectsFutureTimestamp)
{
  rm_localization_adapters::OdometryInputHealthGate gate(enabled_config());
  EXPECT_EQ(
    gate.evaluate(10 * kSecond, 10 * kSecond + 100000000LL, 0.0, 0.0, 0.0),
    rm_localization_adapters::OdometryInputDecision::kRejectFuture);
  EXPECT_TRUE(gate.latched());
}

}  // namespace
