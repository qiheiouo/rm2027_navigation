#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include "rm_navigation_execution_adapters/current_geometry.hpp"
namespace ex = rm_navigation_execution_adapters;
namespace
{
constexpr int64_t epoch = 1820000000123456789;
ex::RawCurrentGrid grid()
{return {80, 80, .05, -2., -2., std::vector<uint8_t>(6400, 0), {"odom", "grid-1", epoch, epoch, true}};}
void obstacle(ex::RawCurrentGrid & g, double x, double y, uint8_t cost)
{
  const int ix = std::floor((x - g.origin_x) / g.resolution), iy = std::floor((y - g.origin_y) / g.resolution);
  g.costs.at(iy * g.width + ix) = cost;
}
ex::CurrentCommandInput input(ex::RawCurrentGrid g = grid())
{
  return {ex::CurrentGridSnapshot(std::move(g)),
    {{{-.34, -.29}, {.34, -.29}, {.34, .29}, {-.34, .29}}, .02, .05, "actual-oldcar-body"},
    {{-.3, -.5}, {.5, .5}, 1.2, 0., 0., 0., 0., "actual-limits"}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.},
    "odom", "base_link", "final-candidate-1", epoch, epoch, epoch, epoch, 50000000};
}
void cover(const ex::GeometryResult & result)
{
  ASSERT_EQ(result.status, ex::GeometryStatus::Certified) << result.reason;
  ASSERT_EQ(result.branches.size(), 2u);
  for (const auto & branch : result.branches) {
    double end = 0.;
    for (const auto & interval : branch.cover) {
      EXPECT_DOUBLE_EQ(interval.begin_seconds, end); EXPECT_GT(interval.margin_lower_bound, 1e-7);
      end = interval.end_seconds;
    }
    EXPECT_DOUBLE_EQ(end, .05);
  }
}
// Independent midpoint integration of body-twist ODE in long double.
ex::Pose oracle(ex::Pose p, ex::Twist v, double duration)
{
  constexpr int steps = 2000; const long double dt = duration / steps;
  long double x = p.x, y = p.y;
  for (int k = 0; k < steps; ++k) {
    const long double yaw = p.yaw + (k + .5L) * dt * v.wz;
    x += dt * (std::cos(yaw) * v.vx - std::sin(yaw) * v.vy);
    y += dt * (std::sin(yaw) * v.vx + std::cos(yaw) * v.vy);
  }
  return {static_cast<double>(x), static_cast<double>(y), p.yaw + duration * v.wz};
}
}

TEST(CurrentGeometry, SameActualInterfaceCoversR4MPPIAndBehaviors)
{
  for (const ex::Twist v : {ex::Twist{.2, -.1, 0.}, {.3, .4, .7}, {0., 0., 1.2},
      {-.3, 0., 0.}, {.15, -.12, -1.2}, {0., 0., 0.}}) {
    auto in = input(); in.measured = in.candidate = v;
    const auto r = ex::inspect_current_command(in); cover(r);
    EXPECT_EQ(r.candidate_identity, in.candidate_identity); EXPECT_EQ(r.map_revision, "grid-1");
    EXPECT_EQ(r.body_revision, in.body.revision); EXPECT_DOUBLE_EQ(r.candidate.wz, v.wz);
    EXPECT_LE(r.elapsed_seconds, .010); EXPECT_LE(r.cell_checks, 50000u);
  }
}
TEST(CurrentGeometry, FilledInteriorClosesInstalledNav2PerimeterGap)
{
  auto g = grid(); obstacle(g, .025, .025, 254);
  nav2_costmap_2d::Costmap2D native(g.width, g.height, g.resolution, g.origin_x, g.origin_y, 0);
  std::copy(g.costs.begin(), g.costs.end(), native.getCharMap());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&native);
  const auto in = input(g); std::vector<geometry_msgs::msg::Point> footprint;
  for (auto p : in.body.padded_footprint) {geometry_msgs::msg::Point v; v.x = p.x; v.y = p.y; footprint.push_back(v);}
  EXPECT_DOUBLE_EQ(checker.footprintCostAtPose(0., 0., 0., footprint), 0.);
  const auto r = ex::inspect_current_command(in);
  EXPECT_EQ(r.status, ex::GeometryStatus::Collision); EXPECT_TRUE(r.branches.empty());
}
TEST(CurrentGeometry, UnknownAndMapBoundaryAreFilledHardObstacles)
{
  auto g = grid(); obstacle(g, .025, .025, 255);
  EXPECT_EQ(ex::inspect_current_command(input(g)).status, ex::GeometryStatus::Collision);
  auto near_boundary = input(); near_boundary.pose.x = -1.8;
  EXPECT_EQ(ex::inspect_current_command(near_boundary).status, ex::GeometryStatus::Collision);
}
TEST(CurrentGeometry, InscribedCentreSemanticsAvoidDoubleFootprintInflation)
{
  auto g = grid(); obstacle(g, .25, .025, 253);
  auto in = input(g); cover(ex::inspect_current_command(in));
  in.pose.x = .275; in.pose.y = .025;
  EXPECT_EQ(ex::inspect_current_command(in).status, ex::GeometryStatus::Collision);
  g = grid(); obstacle(g, .025, .025, 252); cover(ex::inspect_current_command(input(g)));
}
TEST(CurrentGeometry, CentreErrorTubeAlsoProtects253)
{
  auto g = grid(); obstacle(g, .075, .025, 253); auto in = input(g);
  cover(ex::inspect_current_command(in));
  in.bounds.position_error = .08;
  const auto r = ex::inspect_current_command(in);
  EXPECT_EQ(r.status, ex::GeometryStatus::Collision); EXPECT_EQ(r.reason, "centre touches inscribed cell");
}
TEST(CurrentGeometry, HeldBodyTwistEndpointsMatchIndependentIntegration)
{
  for (ex::Twist v : {ex::Twist{.5, .5, 1.2}, {-.3, -.5, -1.2}, {.2, .1, 1e-12}, {.3, -.4, 0.}}) {
    const ex::Pose start{-1.25, .75, 2.4}; const auto actual = ex::integrate_held_body_twist(start, v, .05);
    const auto expected = oracle(start, v, .05);
    EXPECT_NEAR(actual.x, expected.x, 1e-10); EXPECT_NEAR(actual.y, expected.y, 1e-10);
    EXPECT_DOUBLE_EQ(actual.yaw, expected.yaw);
  }
}
TEST(CurrentGeometry, IndependentCurvedOracleStaysInsideChordReserve)
{
  for (ex::Twist v : {ex::Twist{.5, .5, 1.2}, {-.3, -.5, -1.2}, {.2, .1, 1e-12}, {.3, -.4, 0.}}) {
    const ex::Pose start{-.75, .5, -1.2}; const auto end = oracle(start, v, .05);
    const double bound = std::hypot(v.vx, v.vy) * std::abs(v.wz) * .05 * .05 / 8;
    for (int k = 0; k <= 100; ++k) {
      const double alpha = k / 100.; const auto actual = oracle(start, v, .05 * alpha);
      const double chord_x = start.x + alpha * (end.x - start.x), chord_y = start.y + alpha * (end.y - start.y);
      EXPECT_LE(std::hypot(actual.x - chord_x, actual.y - chord_y), bound + 1e-10);
    }
    auto in = input(); in.candidate = v; const auto result = ex::inspect_current_command(in); cover(result);
    EXPECT_NEAR(result.branches[1].chord_error, bound, 1e-16);
  }
}
TEST(CurrentGeometry, Actual50msSpinFindsCollisionBetweenSafeEndpoints)
{
  const double radius = std::hypot(.34, .29), theta = std::atan2(.29, .34);
  const double reserve = std::hypot(.5, .5) * .010 + radius * 1.2 * .010;
  const double wall = radius + .05 + reserve - .00005;
  ex::RawCurrentGrid g{200, 200, .02, wall - 2., -2., std::vector<uint8_t>(40000, 0),
    {"odom", "spin-wall", epoch, epoch, true}};
  for (int y = 0; y < g.height; ++y) {g.costs[y * g.width + 100] = 254;}
  auto in = input(g); in.pose.yaw = theta - .03;
  cover(ex::inspect_current_command(in)); in.pose.yaw = theta + .03; cover(ex::inspect_current_command(in));
  in.pose.yaw = theta - .03; in.measured.wz = in.candidate.wz = 1.2;
  const auto rotating = ex::inspect_current_command(in);
  EXPECT_EQ(rotating.status, ex::GeometryStatus::Collision); EXPECT_TRUE(rotating.branches.empty());
}
TEST(CurrentGeometry, FreshnessMustCoverProcessingAndFullHeldInterval)
{
  auto fresh = input(); fresh.pose_stamp_ns -= 40000000; fresh.tf_stamp_ns = fresh.pose_stamp_ns;
  cover(ex::inspect_current_command(fresh));
  fresh.pose_stamp_ns -= 1; fresh.tf_stamp_ns = fresh.pose_stamp_ns;
  EXPECT_EQ(ex::inspect_current_command(fresh).status, ex::GeometryStatus::Unavailable);
  auto g = grid(); g.metadata.update_ns -= 100000000;
  EXPECT_EQ(ex::inspect_current_command(input(g)).status, ex::GeometryStatus::Unavailable);
  auto in = input(); in.velocity_stamp_ns += 1;
  EXPECT_EQ(ex::inspect_current_command(in).status, ex::GeometryStatus::Unavailable);
}
TEST(CurrentGeometry, BadFrameTFCurrentBodyAndLimitsAreUnavailable)
{
  for (int n = 0; n < 8; ++n) {
    auto in = input();
    if (n == 0) {in.frame = "map";}
    if (n == 1) {in.tf_stamp_ns -= 1;}
    if (n == 2) {auto g = grid(); g.metadata.current = false; in.grid = ex::CurrentGridSnapshot(g);}
    if (n == 3) {in.body.padding = 0.;}
    if (n == 4) {in.candidate.vx = .8;}
    if (n == 5) {in.measured.wz = 1.3;}
    if (n == 6) {in.body.padded_footprint = {{0., 0.}, {1., 0.}, {0., 1.}};}
    if (n == 7) {in.bounds.tracking_yaw_error = std::numeric_limits<double>::quiet_NaN();}
    const auto r = ex::inspect_current_command(in); EXPECT_EQ(r.status, ex::GeometryStatus::Unavailable);
    EXPECT_TRUE(r.branches.empty());
  }
}
TEST(CurrentGeometry, WorkAndTimeExhaustionNeverReturnPartialCover)
{
  for (int n = 0; n < 3; ++n) {
    ex::GeometryPolicy p;
    if (n == 0) {p.max_cell_checks = 1;}
    if (n == 1) {p.max_intervals = 1;}
    if (n == 2) {p.budget_seconds = 1e-12;}
    const auto r = ex::inspect_current_command(input(), p);
    EXPECT_EQ(r.status, ex::GeometryStatus::Unavailable); EXPECT_TRUE(r.branches.empty());
  }
}
TEST(CurrentGeometry, ExplicitTrackingTubeMustContainActualResponse)
{
  auto g = grid(); obstacle(g, .55, .025, 254); auto in = input(g);
  cover(ex::inspect_current_command(in)); in.bounds.tracking_position_error = .2;
  EXPECT_EQ(ex::inspect_current_command(in).status, ex::GeometryStatus::Collision);
}
TEST(CurrentGeometry, SnapshotOwnsRawValuesAndMalformedMetadataCannotIndex)
{
  auto g = grid(); auto in = input(g); obstacle(g, .025, .025, 254);
  cover(ex::inspect_current_command(in));
  g = grid(); g.costs.pop_back(); EXPECT_THROW(ex::CurrentGridSnapshot{g}, std::invalid_argument);
  g = grid(); g.metadata.receipt_ns = epoch - 1; EXPECT_THROW(ex::CurrentGridSnapshot{g}, std::invalid_argument);
}
