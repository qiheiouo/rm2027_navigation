#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

#include "rm_relocalization_bridge/relocalization_math.hpp"

namespace
{

tf2::Transform makeTransform(double x, double y, double z, double yaw)
{
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(x, y, z));
  transform.setRotation(rotation);
  return transform;
}

void expectTransformsNear(
  const tf2::Transform & actual,
  const tf2::Transform & expected,
  double tolerance = 1.0e-9)
{
  EXPECT_NEAR(actual.getOrigin().x(), expected.getOrigin().x(), tolerance);
  EXPECT_NEAR(actual.getOrigin().y(), expected.getOrigin().y(), tolerance);
  EXPECT_NEAR(actual.getOrigin().z(), expected.getOrigin().z(), tolerance);
  EXPECT_NEAR(std::abs(actual.getRotation().dot(expected.getRotation())), 1.0, tolerance);
}

}  // namespace

TEST(RelocalizationMath, RecoversMapToOdomCorrection)
{
  const auto expected_map_to_odom = makeTransform(3.0, -1.0, 0.2, 0.35);
  const auto odom_to_base = makeTransform(1.2, 0.5, -0.1, -0.2);
  const auto map_to_base = expected_map_to_odom * odom_to_base;

  const auto actual = rm_relocalization_bridge::computeMapToOdom(
    map_to_base, odom_to_base);
  expectTransformsNear(actual, expected_map_to_odom);
}

TEST(TimedTransformCache, SelectsNearestSampleWithinTolerance)
{
  rm_relocalization_bridge::TimedTransformCache cache(4U);
  EXPECT_TRUE(cache.add({100, makeTransform(1.0, 0.0, 0.0, 0.0)}));
  EXPECT_TRUE(cache.add({200, makeTransform(2.0, 0.0, 0.0, 0.0)}));
  EXPECT_TRUE(cache.add({300, makeTransform(3.0, 0.0, 0.0, 0.0)}));

  const auto sample = cache.nearest(215, 20);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->stamp_nanoseconds, 200);
  EXPECT_DOUBLE_EQ(sample->transform.getOrigin().x(), 2.0);
  EXPECT_FALSE(cache.nearest(250, 20).has_value());
}

TEST(TimedTransformCache, EvictsOldestSample)
{
  rm_relocalization_bridge::TimedTransformCache cache(2U);
  EXPECT_TRUE(cache.add({100, makeTransform(1.0, 0.0, 0.0, 0.0)}));
  EXPECT_TRUE(cache.add({200, makeTransform(2.0, 0.0, 0.0, 0.0)}));
  EXPECT_TRUE(cache.add({300, makeTransform(3.0, 0.0, 0.0, 0.0)}));
  EXPECT_EQ(cache.size(), 2U);
  EXPECT_FALSE(cache.nearest(100, 0).has_value());
  EXPECT_TRUE(cache.nearest(200, 0).has_value());
}

TEST(TimedTransformCache, ClearsOnTimestampReset)
{
  rm_relocalization_bridge::TimedTransformCache cache(4U);
  EXPECT_TRUE(cache.add({200, makeTransform(2.0, 0.0, 0.0, 0.0)}));
  EXPECT_FALSE(cache.add({100, makeTransform(1.0, 0.0, 0.0, 0.0)}));
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_TRUE(cache.nearest(100, 0).has_value());
  EXPECT_FALSE(cache.nearest(200, 0).has_value());
}

TEST(RelocalizationMath, RejectsNonFiniteTransform)
{
  auto transform = makeTransform(0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(rm_relocalization_bridge::isFiniteTransform(transform));
  transform.setOrigin(tf2::Vector3(NAN, 0.0, 0.0));
  EXPECT_FALSE(rm_relocalization_bridge::isFiniteTransform(transform));
}

TEST(RelocalizationMath, MeasuresPlanarCorrectionInnovation)
{
  const auto previous = makeTransform(1.0, -2.0, 0.5, 0.2);
  const auto candidate = makeTransform(1.3, -1.6, -0.4, 0.5);

  const auto innovation =
    rm_relocalization_bridge::measureCorrectionInnovation(previous, candidate);

  EXPECT_NEAR(innovation.translation_xy_m, 0.5, 1.0e-9);
  EXPECT_NEAR(innovation.yaw_rad, 0.3, 1.0e-9);
}

TEST(RelocalizationMath, MeasuresShortestYawCorrectionAcrossPi)
{
  constexpr double pi = 3.14159265358979323846;
  const auto previous = makeTransform(0.0, 0.0, 0.0, pi - 0.1);
  const auto candidate = makeTransform(0.0, 0.0, 0.0, -pi + 0.1);

  const auto innovation =
    rm_relocalization_bridge::measureCorrectionInnovation(previous, candidate);

  EXPECT_NEAR(innovation.translation_xy_m, 0.0, 1.0e-9);
  EXPECT_NEAR(innovation.yaw_rad, 0.2, 1.0e-9);
}
