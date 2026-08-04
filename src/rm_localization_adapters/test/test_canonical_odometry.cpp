#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "rm_localization_adapters/canonical_odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace
{

constexpr double kPi = 3.14159265358979323846;

tf2::Transform make_transform(double x, double y, double z, double yaw)
{
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

tf2::Transform make_transform_rpy(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  tf2::Quaternion rotation;
  rotation.setRPY(roll, pitch, yaw);
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

rm_localization_adapters::PoseTwistEstimator make_estimator(double alpha = 1.0)
{
  rm_localization_adapters::TwistEstimatorConfig config;
  config.min_dt_sec = 0.001;
  config.max_dt_sec = 2.0;
  config.smoothing_alpha = alpha;
  config.max_linear_speed = 10.0;
  config.max_angular_speed = 20.0;
  return rm_localization_adapters::PoseTwistEstimator(config);
}

TEST(CanonicalOdometry, ComputesBaseTransformInsteadOfRenamingFrame)
{
  const auto odom_to_base = make_transform(1.0, 2.0, 0.0, 0.3);
  const auto base_to_sensor = make_transform(0.0, 0.12, 0.35, 0.4);
  const auto odom_to_sensor = odom_to_base * base_to_sensor;

  const auto recovered = rm_localization_adapters::compute_base_transform(
    odom_to_sensor, base_to_sensor);

  EXPECT_NEAR(recovered.getOrigin().x(), 1.0, 1.0e-9);
  EXPECT_NEAR(recovered.getOrigin().y(), 2.0, 1.0e-9);
  EXPECT_NEAR(recovered.getOrigin().z(), 0.0, 1.0e-9);
  EXPECT_NEAR(recovered.getRotation().angleShortestPath(odom_to_base.getRotation()), 0.0, 1.0e-9);
}

TEST(CanonicalOdometry, SensorInitialModeDoesNotLeakMountRotationAtStartup)
{
  const auto base_to_sensor = make_transform_rpy(
    0.157178, 0.172624, 0.174374, 0.332345, 0.287549, 1.668370);
  tf2::Transform sensor_initial_to_sensor;
  sensor_initial_to_sensor.setIdentity();

  const auto recovered = rm_localization_adapters::compute_base_transform_from_sensor_initial(
    sensor_initial_to_sensor, base_to_sensor);

  EXPECT_NEAR(recovered.getOrigin().x(), 0.0, 1.0e-9);
  EXPECT_NEAR(recovered.getOrigin().y(), 0.0, 1.0e-9);
  EXPECT_NEAR(recovered.getOrigin().z(), 0.0, 1.0e-9);
  EXPECT_NEAR(recovered.getRotation().getAngle(), 0.0, 1.0e-9);
}

TEST(CanonicalOdometry, SensorInitialModeRecoversBaseMotion)
{
  const auto base_to_sensor = make_transform_rpy(
    0.157178, 0.172624, 0.174374, 0.332345, 0.287549, 1.668370);
  const auto base_initial_to_base = make_transform(0.4, -0.2, 0.0, 0.25);
  const auto sensor_initial_to_sensor =
    base_to_sensor.inverse() * base_initial_to_base * base_to_sensor;

  const auto recovered = rm_localization_adapters::compute_base_transform_from_sensor_initial(
    sensor_initial_to_sensor, base_to_sensor);

  EXPECT_NEAR(recovered.getOrigin().x(), 0.4, 1.0e-9);
  EXPECT_NEAR(recovered.getOrigin().y(), -0.2, 1.0e-9);
  EXPECT_NEAR(recovered.getOrigin().z(), 0.0, 1.0e-9);
  EXPECT_NEAR(
    recovered.getRotation().angleShortestPath(base_initial_to_base.getRotation()), 0.0, 1.0e-9);
}

TEST(CanonicalOdometry, ChassisHeadingFusionStartsAtCanonicalIdentity)
{
  const auto initial_base_to_sensor = make_transform_rpy(
    0.0, 0.0, 0.35, 0.25, -0.2, 0.6);
  const auto raw_start = make_transform_rpy(
    3.0, -2.0, 1.0, 0.1, 0.2, -0.3);

  const auto result = rm_localization_adapters::compute_base_transform_from_chassis_heading(
    raw_start, raw_start, initial_base_to_sensor, 2.8, 2.8);

  EXPECT_NEAR(result.base_initial_to_base.getOrigin().length(), 0.0, 1.0e-9);
  EXPECT_NEAR(result.base_initial_to_base.getRotation().getAngle(), 0.0, 1.0e-9);
  EXPECT_NEAR(result.gimbal_yaw_rad, 0.0, 1.0e-9);
}

TEST(CanonicalOdometry, ChassisHeadingFusionRecoversTranslationYawAndGimbalMotion)
{
  const auto initial_base_to_sensor = make_transform_rpy(
    0.0, 0.0, 0.35, 0.25, -0.2, 0.6);
  const auto expected_base = make_transform(1.2, -0.7, 0.0, 0.8);
  const auto gimbal_delta = make_transform(0.0, 0.0, 0.0, -1.1);
  const auto base_to_sensor = gimbal_delta * initial_base_to_sensor;
  const auto sensor_start_to_sensor =
    initial_base_to_sensor.inverse() * expected_base * base_to_sensor;

  tf2::Transform raw_start;
  raw_start.setIdentity();
  const auto result = rm_localization_adapters::compute_base_transform_from_chassis_heading(
    raw_start, sensor_start_to_sensor, initial_base_to_sensor, 2.9, -2.583185307179586,
    0.0);

  EXPECT_NEAR(result.base_initial_to_base.getOrigin().x(), 1.2, 1.0e-9);
  EXPECT_NEAR(result.base_initial_to_base.getOrigin().y(), -0.7, 1.0e-9);
  EXPECT_NEAR(result.base_initial_to_base.getOrigin().z(), 0.0, 1.0e-9);
  EXPECT_NEAR(
    result.base_initial_to_base.getRotation().angleShortestPath(
      expected_base.getRotation()),
    0.0, 1.0e-9);
  EXPECT_NEAR(result.gimbal_yaw_rad, -1.1, 1.0e-9);
}

TEST(CanonicalOdometry, ChassisHeadingFusionHandlesHeadingWrap)
{
  tf2::Transform identity;
  identity.setIdentity();
  const auto result = rm_localization_adapters::compute_base_transform_from_chassis_heading(
    identity, identity, identity, 3.10, -3.10);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(result.base_initial_to_base.getRotation()).getRPY(roll, pitch, yaw);
  static_cast<void>(roll);
  static_cast<void>(pitch);
  EXPECT_NEAR(yaw, 0.083185307179586, 1.0e-9);
}

TEST(PoseTwistEstimator, EstimatesForwardAndLateralVelocity)
{
  auto estimator = make_estimator();
  EXPECT_EQ(
    estimator.update(make_transform(0.0, 0.0, 0.0, 0.0), 1000000000LL).status,
    rm_localization_adapters::TwistEstimateStatus::kInitialized);

  const auto result = estimator.update(
    make_transform(0.4, -0.2, 0.0, 0.0), 1200000000LL);
  ASSERT_EQ(result.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(result.twist.linear.x, 2.0, 1.0e-9);
  EXPECT_NEAR(result.twist.linear.y, -1.0, 1.0e-9);
}

TEST(PoseTwistEstimator, ExpressesLinearVelocityInCurrentBaseFrame)
{
  auto estimator = make_estimator();
  estimator.update(make_transform(0.0, 0.0, 0.0, kPi / 2.0), 1000000000LL);

  const auto result = estimator.update(
    make_transform(0.0, 1.0, 0.0, kPi / 2.0), 2000000000LL);
  ASSERT_EQ(result.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(result.twist.linear.x, 1.0, 1.0e-9);
  EXPECT_NEAR(result.twist.linear.y, 0.0, 1.0e-9);
}

TEST(PoseTwistEstimator, EstimatesYawRate)
{
  auto estimator = make_estimator();
  estimator.update(make_transform(0.0, 0.0, 0.0, 0.0), 1000000000LL);

  const auto result = estimator.update(
    make_transform(0.0, 0.0, 0.0, kPi / 2.0), 2000000000LL);
  ASSERT_EQ(result.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(result.twist.angular.z, kPi / 2.0, 1.0e-9);
}

TEST(PoseTwistEstimator, RejectsNonMonotonicAndLongGapTimestamps)
{
  auto estimator = make_estimator();
  estimator.update(make_transform(0.0, 0.0, 0.0, 0.0), 1000000000LL);
  EXPECT_EQ(
    estimator.update(make_transform(1.0, 0.0, 0.0, 0.0), 900000000LL).status,
    rm_localization_adapters::TwistEstimateStatus::kInvalidTime);
  EXPECT_EQ(
    estimator.update(make_transform(2.0, 0.0, 0.0, 0.0), 4000000000LL).status,
    rm_localization_adapters::TwistEstimateStatus::kInvalidTime);
}

TEST(PoseTwistEstimator, RejectsVelocityOutlierAndRecovers)
{
  auto estimator = make_estimator();
  estimator.update(make_transform(0.0, 0.0, 0.0, 0.0), 1000000000LL);
  EXPECT_EQ(
    estimator.update(make_transform(20.0, 0.0, 0.0, 0.0), 2000000000LL).status,
    rm_localization_adapters::TwistEstimateStatus::kOutlier);

  const auto recovered = estimator.update(
    make_transform(20.5, 0.0, 0.0, 0.0), 3000000000LL);
  ASSERT_EQ(recovered.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(recovered.twist.linear.x, 0.5, 1.0e-9);
}

TEST(PoseTwistEstimator, RejectsNonFiniteVelocity)
{
  auto estimator = make_estimator();
  estimator.update(make_transform(0.0, 0.0, 0.0, 0.0), 1000000000LL);
  const auto result = estimator.update(
    make_transform(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0),
    2000000000LL);
  EXPECT_EQ(result.status, rm_localization_adapters::TwistEstimateStatus::kOutlier);
}

TEST(PoseTwistEstimator, CancelsDynamicGimbalMotionBeforeDifferentiation)
{
  auto estimator = make_estimator();
  const auto odom_to_base = make_transform(0.0, 0.0, 0.0, 0.0);
  const auto gimbal_to_sensor = make_transform(0.0, 0.12, 0.0, 0.0);

  const auto base_to_sensor_at_zero =
    make_transform(0.0, 0.0, 0.35, 0.0) * gimbal_to_sensor;
  const auto first_base = rm_localization_adapters::compute_base_transform(
    odom_to_base * base_to_sensor_at_zero, base_to_sensor_at_zero);
  estimator.update(first_base, 1000000000LL);

  const auto base_to_sensor_rotated =
    make_transform(0.0, 0.0, 0.35, 1.0) * gimbal_to_sensor;
  const auto second_base = rm_localization_adapters::compute_base_transform(
    odom_to_base * base_to_sensor_rotated, base_to_sensor_rotated);
  const auto result = estimator.update(second_base, 1100000000LL);

  ASSERT_EQ(result.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(result.twist.linear.x, 0.0, 1.0e-9);
  EXPECT_NEAR(result.twist.linear.y, 0.0, 1.0e-9);
  EXPECT_NEAR(result.twist.angular.z, 0.0, 1.0e-9);
}

TEST(PoseTwistEstimator, AppliesLowPassSmoothing)
{
  auto estimator = make_estimator(0.25);
  estimator.update(make_transform(0.0, 0.0, 0.0, 0.0), 1000000000LL);
  const auto first = estimator.update(
    make_transform(1.0, 0.0, 0.0, 0.0), 2000000000LL);
  ASSERT_EQ(first.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(first.twist.linear.x, 1.0, 1.0e-9);

  const auto second = estimator.update(
    make_transform(3.0, 0.0, 0.0, 0.0), 3000000000LL);
  ASSERT_EQ(second.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(second.twist.linear.x, 1.25, 1.0e-9);
}

TEST(PoseTwistEstimator, RotatesFilteredTwistIntoCurrentBaseFrame)
{
  auto estimator = make_estimator(0.5);
  estimator.update(make_transform(0.0, 0.0, 0.0, 0.0), 1000000000LL);
  const auto first = estimator.update(
    make_transform(1.0, 0.0, 0.0, 0.0), 2000000000LL);
  ASSERT_EQ(first.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(first.twist.linear.x, 1.0, 1.0e-9);

  const auto second = estimator.update(
    make_transform(1.0, 1.0, 0.0, kPi / 2.0), 3000000000LL);
  ASSERT_EQ(second.status, rm_localization_adapters::TwistEstimateStatus::kValid);
  EXPECT_NEAR(second.twist.linear.x, 0.5, 1.0e-9);
  EXPECT_NEAR(second.twist.linear.y, -0.5, 1.0e-9);
}

}  // namespace
