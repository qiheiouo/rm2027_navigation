#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rm_mid360_driver_bridge/ramp_plane_filter.hpp"

namespace
{

using rm_mid360_driver_bridge::Point2d;
using rm_mid360_driver_bridge::Point3d;
using rm_mid360_driver_bridge::RampRegion;

RampRegion make_region(double slope_deg)
{
  RampRegion region;
  region.name = "test_ramp";
  region.polygon = {
    Point2d{0.0, -0.5}, Point2d{1.0, -0.5},
    Point2d{1.0, 0.5}, Point2d{0.0, 0.5}};
  region.origin_x = 0.0;
  region.origin_y = 0.0;
  region.origin_height = 0.0;
  region.ascent_yaw_rad = 0.0;
  region.slope_rad = slope_deg * 3.14159265358979323846 / 180.0;
  region.surface_tolerance = 0.04;
  return region;
}

std::vector<Point3d> make_unlabelled_ramp(double slope_deg)
{
  std::vector<Point3d> points;
  const double gradient = std::tan(slope_deg * 3.14159265358979323846 / 180.0);
  for (int x_index = 0; x_index <= 15; ++x_index) {
    for (int y_index = -6; y_index <= 6; ++y_index) {
      const double x = 0.10 * static_cast<double>(x_index);
      const double y = 0.10 * static_cast<double>(y_index);
      const double noise = 0.002 * static_cast<double>((x_index + y_index + 30) % 3 - 1);
      points.push_back(Point3d{x, y, gradient * x + noise});
    }
  }
  for (int x_index = -5; x_index < 0; ++x_index) {
    for (int y_index = -6; y_index <= 6; ++y_index) {
      points.push_back(
        Point3d{
          0.10 * static_cast<double>(x_index),
          0.10 * static_cast<double>(y_index),
          0.0});
    }
  }
  return points;
}

}  // namespace

TEST(RampPlaneFilter, ComputesCompetitionCandidateHeights)
{
  const auto ramp_11 = make_region(11.0);
  const auto ramp_15 = make_region(15.0);
  EXPECT_NEAR(
    rm_mid360_driver_bridge::expected_ramp_height(ramp_11, 1.0, 0.0),
    0.19438, 1e-4);
  EXPECT_NEAR(
    rm_mid360_driver_bridge::expected_ramp_height(ramp_15, 1.0, 0.0),
    0.26795, 1e-4);
}

TEST(RampPlaneFilter, RemovesOnlyExpectedSurfaceInsidePolygon)
{
  const auto region = make_region(15.0);
  const std::vector<RampRegion> regions{region};
  const double surface =
    rm_mid360_driver_bridge::expected_ramp_height(region, 0.60, 0.10);

  EXPECT_TRUE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      regions, 0.60, 0.10, surface + 0.02).has_value());
  EXPECT_FALSE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      regions, 0.60, 0.10, surface + 0.08).has_value());
  EXPECT_FALSE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      regions, 1.20, 0.10, surface).has_value());
}

TEST(RampPlaneFilter, IncludesPolygonBoundaryAndSupportsRotatedAscent)
{
  auto region = make_region(11.0);
  region.polygon = {
    Point2d{-0.5, 0.0}, Point2d{0.5, 0.0},
    Point2d{0.5, 1.0}, Point2d{-0.5, 1.0}};
  region.ascent_yaw_rad = 0.5 * 3.14159265358979323846;
  const double expected =
    rm_mid360_driver_bridge::expected_ramp_height(region, 0.0, 1.0);
  EXPECT_NEAR(expected, std::tan(region.slope_rad), 1e-9);
  EXPECT_TRUE(
    rm_mid360_driver_bridge::point_in_polygon(
      region.polygon, -0.5, 0.5));
  EXPECT_TRUE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      std::vector<RampRegion>{region}, 0.0, 1.0, expected).has_value());
}

TEST(RampPlaneFilter, RejectsInvalidRegionContracts)
{
  std::string error;
  auto region = make_region(15.0);
  EXPECT_TRUE(rm_mid360_driver_bridge::validate_ramp_region(region, error));
  EXPECT_TRUE(error.empty());

  region.polygon = {Point2d{0.0, 0.0}, Point2d{1.0, 0.0}};
  EXPECT_FALSE(rm_mid360_driver_bridge::validate_ramp_region(region, error));

  region = make_region(15.0);
  region.surface_tolerance = 0.30;
  EXPECT_FALSE(rm_mid360_driver_bridge::validate_ramp_region(region, error));

  region = make_region(50.0);
  EXPECT_FALSE(rm_mid360_driver_bridge::validate_ramp_region(region, error));
}

TEST(RampPlaneFilter, KeepsNonFiniteReturnsFailClosed)
{
  const std::vector<RampRegion> regions{make_region(11.0)};
  EXPECT_FALSE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      regions, NAN, 0.0, 0.0).has_value());
  EXPECT_FALSE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      regions, 0.5, 0.0, INFINITY).has_value());
}

TEST(RampPlaneFilter, DetectsUnlabelledTraversablePlane)
{
  const auto points = make_unlabelled_ramp(11.0);
  rm_mid360_driver_bridge::AutomaticRampDetectionConfig config;
  const auto detection =
    rm_mid360_driver_bridge::detect_automatic_ramp(points, config);
  ASSERT_TRUE(detection.has_value());
  EXPECT_NEAR(
    detection->region.slope_rad,
    11.0 * 3.14159265358979323846 / 180.0,
    0.01);
  EXPECT_GT(detection->length, 1.35);
  EXPECT_GT(detection->width, 1.05);
  EXPECT_GT(detection->inlier_ratio, 0.70);
  EXPECT_LT(detection->rms_residual, 0.01);
}

TEST(RampPlaneFilter, AutomaticDetectionKeepsObstacleAbovePlane)
{
  auto points = make_unlabelled_ramp(15.0);
  points.push_back(
    Point3d{0.80, 0.0,
      std::tan(15.0 * 3.14159265358979323846 / 180.0) * 0.80 + 0.15});
  rm_mid360_driver_bridge::AutomaticRampDetectionConfig config;
  const auto detection =
    rm_mid360_driver_bridge::detect_automatic_ramp(points, config);
  ASSERT_TRUE(detection.has_value());
  const std::vector<RampRegion> regions{detection->region};
  const double surface = rm_mid360_driver_bridge::expected_ramp_height(
    detection->region, 0.80, 0.0);
  EXPECT_TRUE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      regions, 0.80, 0.0, surface + 0.02).has_value());
  EXPECT_FALSE(
    rm_mid360_driver_bridge::match_expected_ramp_surface(
      regions, 0.80, 0.0, surface + 0.15).has_value());
}

TEST(RampPlaneFilter, AutomaticDetectionRejectsFlatAndNarrowSurfaces)
{
  std::vector<Point3d> flat;
  for (int x_index = 0; x_index <= 15; ++x_index) {
    for (int y_index = -6; y_index <= 6; ++y_index) {
      flat.push_back(
        Point3d{
        0.10 * static_cast<double>(x_index),
        0.10 * static_cast<double>(y_index),
        0.0});
    }
  }
  rm_mid360_driver_bridge::AutomaticRampDetectionConfig config;
  EXPECT_FALSE(rm_mid360_driver_bridge::detect_automatic_ramp(flat, config).has_value());

  auto narrow = make_unlabelled_ramp(11.0);
  narrow.erase(
    std::remove_if(
      narrow.begin(), narrow.end(),
      [](const Point3d & point) {return std::abs(point.y) > 0.15;}),
    narrow.end());
  EXPECT_FALSE(rm_mid360_driver_bridge::detect_automatic_ramp(narrow, config).has_value());
}
