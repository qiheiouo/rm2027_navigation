#include <cmath>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "rm_dog_hole/geometry.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

rm_dog_hole::Corridor makeCorridor()
{
  rm_dog_hole::Corridor corridor;
  corridor.center_x = -3.0;
  corridor.center_y = 0.0;
  corridor.yaw = kPi;
  corridor.width = 0.8;
  corridor.length = 1.0;
  return corridor;
}

TEST(DogHoleGeometry, ReportsNominalClearanceAtCenter)
{
  const auto pose = rm_dog_hole::evaluatePose(
    -3.0, 0.0, kPi, makeCorridor(), 0.6, 0.5);
  EXPECT_NEAR(pose.longitudinal, 0.0, 1e-9);
  EXPECT_NEAR(pose.lateral, 0.0, 1e-9);
  EXPECT_NEAR(pose.heading_error, 0.0, 1e-9);
  EXPECT_NEAR(pose.minimum_wall_clearance, 0.15, 1e-9);
}

TEST(DogHoleGeometry, UsesBaseHeadingRatherThanSensorHeading)
{
  const auto pose = rm_dog_hole::evaluatePose(
    -3.0, 0.0, kPi - 0.2, makeCorridor(), 0.6, 0.5);
  EXPECT_NEAR(pose.heading_error, 0.2, 1e-9);
}

TEST(DogHoleGeometry, IncludesRotatedRectangularCornersInClearance)
{
  constexpr double heading_error = 10.0 * kPi / 180.0;
  const auto pose = rm_dog_hole::evaluatePose(
    -3.0, 0.0, kPi - heading_error, makeCorridor(), 0.6, 0.5);
  const double expected_half_width =
    0.25 * std::cos(heading_error) + 0.30 * std::sin(heading_error);
  EXPECT_NEAR(
    pose.minimum_wall_clearance,
    0.4 - expected_half_width,
    1e-9);
}

rm_dog_hole::Footprint makeAlternatingEdgeOctagon()
{
  constexpr double long_edge = 0.382;
  constexpr double chamfer_edge = 0.126;
  const double across_flats = long_edge + std::sqrt(2.0) * chamfer_edge;
  constexpr double half_long_edge = 0.5 * long_edge;
  const double half_across_flats = 0.5 * across_flats;
  return {
    {half_across_flats, half_long_edge},
    {half_long_edge, half_across_flats},
    {-half_long_edge, half_across_flats},
    {-half_across_flats, half_long_edge},
    {-half_across_flats, -half_long_edge},
    {-half_long_edge, -half_across_flats},
    {half_long_edge, -half_across_flats},
    {half_across_flats, -half_long_edge},
  };
}

TEST(DogHoleGeometry, UsesExactOctagonalFootprintAtCorridorHeading)
{
  const double across_flats = 0.382 + std::sqrt(2.0) * 0.126;
  const auto pose = rm_dog_hole::evaluatePose(
    -3.0, 0.0, kPi, makeCorridor(), makeAlternatingEdgeOctagon());
  EXPECT_NEAR(pose.minimum_wall_clearance, 0.4 - 0.5 * across_flats, 1e-9);
}

TEST(DogHoleGeometry, OctagonalClearanceChangesWithTraversalDirection)
{
  constexpr double long_edge = 0.382;
  const double across_flats = long_edge + std::sqrt(2.0) * 0.126;
  const auto pose = rm_dog_hole::evaluatePose(
    -3.0, 0.0, kPi + 0.25 * kPi,
    makeCorridor(), makeAlternatingEdgeOctagon());
  const double expected_half_width =
    0.5 * (across_flats + long_edge) / std::sqrt(2.0);
  EXPECT_NEAR(pose.minimum_wall_clearance, 0.4 - expected_half_width, 1e-9);
}

TEST(DogHoleGeometry, SupportsRequiredTraversalYawOffset)
{
  auto corridor = makeCorridor();
  corridor.traversal_yaw_offset = 0.5 * kPi;
  const auto pose = rm_dog_hole::evaluatePose(
    -3.0, 0.0, corridor.yaw + 0.5 * kPi,
    corridor, makeAlternatingEdgeOctagon());
  EXPECT_NEAR(pose.heading_error, 0.0, 1e-9);
}

TEST(DogHoleGeometry, DetectsAPathAcrossBothHalves)
{
  const std::vector<std::pair<double, double>> path{
    {-1.0, 0.0}, {-2.6, 0.0}, {-3.0, 0.0}, {-3.4, 0.0}, {-5.0, 0.0}};
  EXPECT_TRUE(rm_dog_hole::pathCrossesCorridor(
      path, makeCorridor(), 0.8, 0.5));
}

TEST(DogHoleGeometry, RejectsAPathThatPassesOutsideTheWalls)
{
  const std::vector<std::pair<double, double>> path{
    {-1.0, 1.0}, {-3.0, 1.0}, {-5.0, 1.0}};
  EXPECT_FALSE(rm_dog_hole::pathCrossesCorridor(
      path, makeCorridor(), 0.8, 0.5));
}

TEST(DogHoleGeometry, KeepsFullRobotBeforeAFlatEntrance)
{
  EXPECT_NEAR(
    rm_dog_hole::requiredAlignmentOffset(0.6, 0.0, 0.0, 0.05),
    0.35,
    1e-9);
}

TEST(DogHoleGeometry, IncludesEntryRampRunInAlignmentOffset)
{
  const double angle = 11.0 * kPi / 180.0;
  EXPECT_NEAR(
    rm_dog_hole::requiredAlignmentOffset(0.6, 0.06, angle, 0.05),
    0.35 + 0.06 / std::tan(angle),
    1e-9);
}

}  // namespace
