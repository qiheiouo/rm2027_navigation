#include <gtest/gtest.h>
#include <limits>
#include <cmath>
#include "fixtures.hpp"
using namespace r4_test;

TEST(Corridor, ExistingSfcEndpointsAndNegativeOriginAreAdapted)
{
  const auto route = PreparedCorridor::prepare(path(), grid(), body(), 4);
  ASSERT_EQ(route.centre_bounds().size(), 3u); ASSERT_EQ(route.path_indices().size(), 3u);
  EXPECT_EQ(route.generation(), 4u); EXPECT_EQ(route.frame(), "map");
  EXPECT_DOUBLE_EQ(route.arcs().back(), 2.); EXPECT_DOUBLE_EQ(route.project({.3, .2}), 1.3);
  EXPECT_DOUBLE_EQ(route.sample(1.5).position.x, .5);
  for (size_t i = 0; i < route.points().size(); ++i) {
    const auto b = route.centre_bounds()[i]; const auto p = route.points()[i];
    EXPECT_LT(b.xmin, p.x); EXPECT_GT(b.xmax, p.x);
    EXPECT_LT(b.ymin, p.y); EXPECT_GT(b.ymax, p.y);
  }
}
TEST(Corridor, TranslationPreservesBoundsWithoutVendorOriginConvention)
{
  const auto a = PreparedCorridor::prepare(path(), grid(), body(), 1);
  const auto b = PreparedCorridor::prepare(path(3.25, -1.75), grid(-1.75, -5.75), body(), 1);
  for (size_t i = 0; i < a.points().size(); ++i) {
    EXPECT_NEAR(b.centre_bounds()[i].xmin - a.centre_bounds()[i].xmin, 3.25, 1e-6);
    EXPECT_NEAR(b.centre_bounds()[i].ymax - a.centre_bounds()[i].ymax, -1.75, 1e-6);
  }
  EXPECT_NE(a.path_digest(), b.path_digest()); EXPECT_NE(a.map_digest(), b.map_digest());
}
TEST(Corridor, ValuesAndOriginalPathIndicesRemainImmutable)
{
  auto p = path(); p.poses.insert(p.poses.begin() + 1, p.poses.front()); auto g = grid();
  const auto r = PreparedCorridor::prepare(p, g, body(), 1);
  ASSERT_EQ(r.points().size(), 3u); EXPECT_EQ(r.path_indices(), (std::vector<size_t>{0, 2, 3}));
  p.poses[0].pose.position.x = 10.; g.data[100] = 100;
  EXPECT_DOUBLE_EQ(r.points()[0].x, -1.); EXPECT_DOUBLE_EQ(r.arcs().back(), 2.);
}
TEST(Corridor, UnknownAndNonzeroRawOccupancyAreForbidden)
{
  for (int value : {-1, 1, 100}) {
    auto g = grid(); const int x = (0. - g.info.origin.position.x) / g.info.resolution;
    const int y = (0. - g.info.origin.position.y) / g.info.resolution;
    g.data[y * g.info.width + x] = value;
    EXPECT_THROW(PreparedCorridor::prepare(path(), g, body(), 1), ContractError);
  }
}
TEST(Corridor, MechanicalSupportChecksWallAndMapBoundary)
{
  auto g = grid();
  const int wall = std::floor((.15 - g.info.origin.position.x) / g.info.resolution);
  for (size_t y = 0; y < g.info.height; ++y) {g.data[y * g.info.width + wall] = 100;}
  EXPECT_THROW(PreparedCorridor::prepare(path(), g, body(), 1), ContractError);
  auto boundary = path();
  for (auto & p : boundary.poses) {p.pose.position.y = -3.95;}
  EXPECT_THROW(PreparedCorridor::prepare(boundary, grid(), body(), 1), ContractError);
}
TEST(Corridor, FrameRotationMalformedMapAndOutsidePathAreRejected)
{
  auto g = grid(); g.header.frame_id = "odom";
  EXPECT_THROW(PreparedCorridor::prepare(path(), g, body(), 1), ContractError);
  g = grid(); g.info.origin.orientation.z = .1;
  EXPECT_THROW(PreparedCorridor::prepare(path(), g, body(), 1), ContractError);
  g = grid(); g.data.pop_back();
  EXPECT_THROW(PreparedCorridor::prepare(path(), g, body(), 1), ContractError);
  auto p = path(); p.poses[0].pose.position.x = -5.01;
  EXPECT_THROW(PreparedCorridor::prepare(p, grid(), body(), 1), ContractError);
  EXPECT_THROW(PreparedCorridor::prepare(path(), grid(), body(), 1,
    std::numeric_limits<double>::infinity()), ContractError);
}
TEST(Corridor, PlanMapAndActualBodyPolicyDigestsInvalidatePreparedIdentity)
{
  auto p = path(); auto g = grid(); const auto a = PreparedCorridor::prepare(p, g, body(), 1);
  p.poses[1].pose.position.y += .1;
  EXPECT_NE(a.path_digest(), PreparedCorridor::prepare(p, g, body(), 1).path_digest());
  g.data[0] = -1;
  EXPECT_NE(a.map_digest(), PreparedCorridor::prepare(path(), g, body(), 1).map_digest());
  auto b = body(); b.padding += .01;
  EXPECT_NE(a.policy_digest(), PreparedCorridor::prepare(path(), grid(), b, 1).policy_digest());
}

TEST(Corridor, SparseSegmentsNeedContinuousSupportWithoutAnotherPlanner)
{
  auto p = path(); p.poses.erase(p.poses.begin() + 1);
  p.poses.front().pose.position.x = -2.; p.poses.back().pose.position.x = 2.;
  EXPECT_THROW(PreparedCorridor::prepare(p, grid(), body(), 1), ContractError);
  const auto r = PreparedCorridor::prepare(path(), grid(), body(), 1);
  EXPECT_NO_THROW(r.local_bounds({0., 0.}));
  EXPECT_THROW(r.local_bounds({0., 3.}), ContractError);
}
