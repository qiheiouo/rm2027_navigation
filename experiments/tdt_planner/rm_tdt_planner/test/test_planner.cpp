#include "rm_tdt_planner/planner.hpp"
#include "YAstar/yastar.hpp"
#include "MinimumSnapOsqp/minimumSnap.hpp"
#include "benchmark_geometry.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <random>

using namespace rm_tdt_planner;
namespace
{
Grid empty_grid()
{
  return {100, 80, 0.1, -3.25, -1.75, std::vector<uint8_t>(8000, 0)};
}
Point at(const Grid & g, double x, double y) {return {g.origin_x + x, g.origin_y + y};}
Options test_options()
{
  Options o; o.radius = 0.20; o.clearance = 0.02; o.time_budget = 5.0;
  return o;
}
void verify(const Grid & g, Point start, Point goal, const Options & o, const Result & r)
{
  ASSERT_TRUE(r.success) << r.reason;
  ASSERT_FALSE(r.path.empty());
  EXPECT_NEAR(r.path.front().x, start.x, 1e-7);
  EXPECT_NEAR(r.path.front().y, start.y, 1e-7);
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-7);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-7);
  EXPECT_TRUE(collision_free(g, r.path, o));
  for (size_t i = 1; i < r.path.size(); ++i) {
    EXPECT_LE(std::hypot(r.path[i].x-r.path[i-1].x, r.path[i].y-r.path[i-1].y),
      o.output_spacing + 1e-6);
  }
}
}

TEST(Planner, OpenMapActuallyRunsCorridorBackend)
{
  auto g = empty_grid(); auto o = test_options();
  auto s = at(g, 1.13, 1.27), t = at(g, 8.31, 6.24);
  auto r = plan(g, s, t, o);
  verify(g, s, t, o, r);
  EXPECT_TRUE(r.optimized) << r.reason;
}
TEST(Planner, PreparedSnapshotMatchesRawPlanningWithoutDoubleInflation)
{
  auto g = empty_grid(); auto o = test_options();
  for (int y = 0; y < 55; ++y) {g.costs[y*g.width+50] = 254;}
  const auto s = at(g, 1.13, 1.27), t = at(g, 8.31, 2.24);
  auto prepared = prepare_grid(g, o);
  const auto raw = plan(g, s, t, o), cached = plan_prepared(prepared, s, t, o);
  ASSERT_TRUE(raw.success); ASSERT_TRUE(cached.success);
  EXPECT_EQ(raw.optimized, cached.optimized);
  ASSERT_EQ(raw.path.size(), cached.path.size());
  for (size_t i = 0; i < raw.path.size(); ++i) {
    EXPECT_DOUBLE_EQ(raw.path[i].x, cached.path[i].x);
    EXPECT_DOUBLE_EQ(raw.path[i].y, cached.path[i].y);
  }
  EXPECT_TRUE(collision_free_prepared(prepared, cached.path));
  g.costs.assign(g.costs.size(), 254);
  EXPECT_TRUE(plan_prepared(prepared, s, t, o).success);  // immutable old snapshot
  EXPECT_FALSE(plan(g, s, t, o).success);
  o.radius += 0.1;
  const auto mismatch = plan_prepared(prepared, s, t, o);
  EXPECT_FALSE(mismatch.success);
  EXPECT_TRUE(mismatch.path.empty());
  EXPECT_NE(mismatch.reason.find("footprint"), std::string::npos);
}
TEST(BenchmarkGeometry, ExactSegmentDistanceAndSweptDisk)
{
  using benchmark_geometry::segment_box;
  EXPECT_DOUBLE_EQ(segment_box({0,0},{4,4},1,1,2,2),0.0);
  EXPECT_DOUBLE_EQ(segment_box({0,0},{0,4},1,1,2,2),1.0);
  EXPECT_NEAR(segment_box({0,0},{0,0},1,1,2,2),std::sqrt(2.0),1e-12);
  EXPECT_DOUBLE_EQ(segment_box({0,2},{4,2},1,1,2,2),0.0);
  Grid g{10,10,1.0,0,0,std::vector<uint8_t>(100,0)};
  g.costs[5*10+5]=254;
  EXPECT_NEAR(benchmark_geometry::clearance(g,{{3,3},{7,3}},1.5),0.25,1e-12);
  EXPECT_NEAR(benchmark_geometry::clearance(g,{{3,4},{7,4}},1.5),-0.5,1e-12);
  EXPECT_NEAR(benchmark_geometry::clearance(g,{{3,4},{7,4}},1.0),0.0,1e-12);
}
TEST(Planner, DetoursAroundWallAndPreservesOriginAndEndpoints)
{
  auto g = empty_grid(); auto o = test_options();
  for (int y = 0; y < 55; ++y) {g.costs[y*g.width+50] = 254;}
  auto s = at(g, 1.1, 1.2), t = at(g, 8.3, 2.3);
  auto r = plan(g, s, t, o); verify(g, s, t, o, r);
  for (auto p : r.path) {
    // Independent geometric oracle: path centres stay outside the wall's
    // continuous rectangle expanded by the real circular footprint.
    const double x = p.x-g.origin_x, y = p.y-g.origin_y;
    const double dx = std::max({5.0-x, 0.0, x-5.1});
    const double dy = std::max({0.0-y, 0.0, y-5.5});
    EXPECT_GT(std::hypot(dx, dy), o.radius);
  }
}
TEST(Planner, CompletelySealedWallFailsEmpty)
{
  auto g = empty_grid(); auto o = test_options();
  for (int y = 0; y < g.height; ++y) {g.costs[y*g.width+50] = 254;}
  auto r = plan(g, at(g, 1, 1), at(g, 8, 1), o);
  EXPECT_FALSE(r.success); EXPECT_TRUE(r.path.empty());
}
TEST(Planner, UnknownAndInscribedCellsAreBlocked)
{
  for (uint8_t cost : {253, 255}) {
    auto g = empty_grid(); auto o = test_options();
    for (int y = 0; y < g.height; ++y) {g.costs[y*g.width+50] = cost;}
    auto r = plan(g, at(g, 1, 1), at(g, 8, 1), o);
    EXPECT_FALSE(r.success); EXPECT_TRUE(r.path.empty());
  }
}
TEST(Planner, SameCellAndSamePose)
{
  auto g = empty_grid(); auto o = test_options();
  auto s = at(g, 2.22, 2.21), t = at(g, 2.28, 2.27);
  verify(g, s, t, o, plan(g, s, t, o));
  verify(g, s, s, o, plan(g, s, s, o));
}
TEST(Planner, BoundaryNaNAndOccupiedEndpointsFailEmpty)
{
  auto g = empty_grid(); auto o = test_options();
  const auto s = at(g, 2, 2);
  for (Point t : {at(g, -0.01, 2), at(g, 10.0, 2), at(g, 2, 8.0),
      at(g, 0.1, 2), Point{std::numeric_limits<double>::quiet_NaN(), 0}})
  {
    const auto r = plan(g, s, t, o);
    EXPECT_FALSE(r.success); EXPECT_TRUE(r.path.empty());
  }
  g.costs[20*g.width+20] = 254;
  EXPECT_FALSE(plan(g, s, at(g, 8, 2), o).success);
}
TEST(Planner, NarrowPassageRejectedForCircumscribedFootprint)
{
  auto g = empty_grid(); auto o = test_options(); o.radius = 0.45;
  for (int y = 0; y < g.height; ++y) {
    if (y < 37 || y > 43) {g.costs[y*g.width+50] = 254;}
  }
  EXPECT_FALSE(plan(g, at(g, 1, 4), at(g, 8, 4), o).success);
}
TEST(Planner, LateOutputIsNotPublished)
{
  auto g = empty_grid(); auto o = test_options(); o.time_budget = 0.001;
  g.width = 1000; g.height = 800; g.costs.assign(800000, 0);
  auto r = plan(g, at(g, 1, 1), at(g, 90, 70), o);
  EXPECT_FALSE(r.success); EXPECT_TRUE(r.path.empty());
  EXPECT_NE(r.reason.find("deadline"), std::string::npos);
}
TEST(Planner, InvalidConfigurationAndMalformedMapsFailEmpty)
{
  auto g = empty_grid(); auto o = test_options();
  g.costs.pop_back(); EXPECT_FALSE(plan(g, {}, {}, o).success);
  g = empty_grid(); g.resolution = 0; EXPECT_FALSE(plan(g, {}, {}, o).success);
  g = empty_grid(); o.radius = -1; EXPECT_FALSE(plan(g, {}, {}, o).success);
  o = test_options(); o.collision_iterations = 20;
  EXPECT_FALSE(plan(g, {}, {}, o).success);
}
TEST(Planner, FrontendOnlyIsSelectable)
{
  auto g = empty_grid(); auto o = test_options(); o.optimize = false;
  auto s = at(g, 1.2, 1.2), t = at(g, 8.1, 6.1);
  auto r = plan(g, s, t, o); verify(g, s, t, o, r);
  EXPECT_FALSE(r.optimized);
}
TEST(Planner, WaypointBudgetFallsBackToValidatedAstar)
{
  auto g = empty_grid(); auto o = test_options(); o.max_waypoints = 2;
  for (int y = 0; y < 55; ++y) {g.costs[y*g.width+50] = 254;}
  auto s = at(g, 1.1, 1.2), t = at(g, 8.3, 2.3);
  auto r = plan(g, s, t, o); verify(g, s, t, o, r);
  EXPECT_FALSE(r.optimized);
  EXPECT_EQ(r.reason, "validated A* fallback: waypoint or time budget");
}
TEST(Planner, UnresolvedOptimizerCollisionFallsBack)
{
  auto g = empty_grid(); auto o = test_options();
  o.collision_iterations = 1; o.corridor_range = 5.0;
  // Fixed obstacle fixture from seed 20260912, trial 1. One QP collision
  // iteration cannot produce an accepted curve, but a valid A* path exists.
  for (int index : {4271,5934,4067,3157,1633,4841,2328,5833,4064,4074,5233,
      5539,5062,6349,5874,5226,6150,2553,3329,2061,3662,6272,6332,4936,4135})
  {
    g.costs[index] = 254;
  }
  auto s = at(g, 1.13, 1.17), t = at(g, 8.11, 6.19);
  auto r = plan(g, s, t, o); verify(g, s, t, o, r);
  EXPECT_FALSE(r.optimized);
  EXPECT_EQ(r.reason.find("validated A* fallback: optimizer"), 0u);
}
TEST(Planner, IndependentSegmentsCatchWallBetweenSparseSamples)
{
  auto g = empty_grid(); auto o = test_options();
  g.costs[40*g.width+50] = 254;
  EXPECT_FALSE(collision_free(g, {at(g, 2, 4.05), at(g, 8, 4.05)}, o));
}
TEST(Planner, RepeatedCallsDoNotReuseStaleFreeSpace)
{
  auto g = empty_grid(); auto o = test_options();
  auto s = at(g, 1, 1), t = at(g, 8, 1);
  ASSERT_TRUE(plan(g, s, t, o).success);
  for (int y = 0; y < g.height; ++y) {g.costs[y*g.width+50] = 254;}
  EXPECT_FALSE(plan(g, s, t, o).success);
}
TEST(UpstreamFix, BoundsAndDiagonalCorner)
{
  YAstar a(5, 5, 1.0f, 0, 0);
  std::vector<uint8_t> map(25, 255); map[1] = 0; map[5] = 0;
  a.setMap(5, 5, map.data());
  EXPECT_TRUE(a.search({-0.1f, 0.1f}, {3.1f, 3.1f}).empty());
  EXPECT_TRUE(a.search({0.1f, 0.1f}, {5.0f, 2.1f}).empty());
  EXPECT_TRUE(a.search({0.1f, 0.1f}, {1.1f, 1.1f}).empty());
}
TEST(UpstreamFix, ImmutableHeapMatchesIndependentDijkstraCost)
{
  // Independent shortest-cost oracle exercises repeated improvements to open nodes.
  std::mt19937 rng(42901);
  constexpr int n = 30;
  for (int trial = 0; trial < 10; ++trial) {
    std::vector<uint8_t> map(n*n, 255);
    for (int k = 0; k < 90; ++k) {map[rng()%(n*n)] = 0;}
    map[0] = 255; map.back() = 255;
    YAstar a(n, n, 1.0f, 0, 0);
    a.setMap(n, n, map.data());
    a.setCostField(1.0f, [](float d) {return 2.0f/(0.1f+d);});
    a.initCostMap();
    const auto costs = a.getCostMap();
    std::vector<double> dist(n*n, std::numeric_limits<double>::infinity());
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    dist[0] = 0; queue.emplace(0, 0);
    while (!queue.empty()) {
      auto [d, index] = queue.top(); queue.pop();
      if (d != dist[index]) {continue;}
      int x = index%n, y = index/n;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          int nx=x+dx, ny=y+dy;
          if ((!dx && !dy) || nx<0 || ny<0 || nx>=n || ny>=n || !map[ny*n+nx]) {continue;}
          if (dx && dy && (!map[y*n+nx] || !map[ny*n+x])) {continue;}
          double next=d+costs(ny,nx)*std::hypot(dx,dy);
          if (next < dist[ny*n+nx]) {dist[ny*n+nx]=next; queue.emplace(next,ny*n+nx);}
        }
      }
    }
    auto path = a.search({0.5f, 0.5f}, {n-0.5f, n-0.5f});
    if (!std::isfinite(dist.back())) {EXPECT_TRUE(path.empty()); continue;}
    ASSERT_FALSE(path.empty()); double total=0;
    for (size_t i=1; i<path.size(); ++i) {
      total += costs(static_cast<int>(path[i].y()),static_cast<int>(path[i].x())) *
        (path[i]-path[i-1]).norm();
    }
    EXPECT_NEAR(total, dist.back(), 1e-3);
  }
}
TEST(Planner, DeterministicRandomObstacleSafety)
{
  std::mt19937 rng(20260912);
  auto o = test_options(); int successes = 0;
  for (int trial = 0; trial < 20; ++trial) {
    auto g = empty_grid();
    for (int k = 0; k < 25; ++k) {
      int x = 25 + rng()%50, y = 15 + rng()%50;
      g.costs[y*g.width+x] = 254;
    }
    auto s = at(g, 1.13, 1.17), t = at(g, 8.11, 6.19);
    auto r = plan(g, s, t, o);
    if (!r.success) {EXPECT_TRUE(r.path.empty()); continue;}
    ++successes; verify(g, s, t, o, r);
    // Independent point-to-occupied-square oracle, separate from DDA/EDT code.
    for (const auto & p : r.path) {
      double x = p.x-g.origin_x, y = p.y-g.origin_y;
      for (int cy = 0; cy < g.height; ++cy) {
        for (int cx = 0; cx < g.width; ++cx) {
          if (g.costs[cy*g.width+cx] < 253) {continue;}
          double dx = std::max({cx*g.resolution-x, 0.0, x-(cx+1)*g.resolution});
          double dy = std::max({cy*g.resolution-y, 0.0, y-(cy+1)*g.resolution});
          ASSERT_GT(std::hypot(dx, dy), o.radius + o.clearance);
        }
      }
    }
  }
  EXPECT_GE(successes, 10);
}

namespace
{
Grid nav2_wall_grid()
{
  auto g = empty_grid();
  g.cost_interpretation = CostInterpretation::Nav2Master;
  for (int y = 0; y < g.height; ++y) {
    for (int x = 48; x <= 52; ++x) {g.costs[y * g.width + x] = x == 50 ? 254 : 253;}
  }
  return g;
}
}

TEST(Nav2CostSemantics, InscribedBandDoesNotSeedASecondFootprintInflation)
{
  const auto g = nav2_wall_grid();
  auto o = test_options();
  const auto start = at(g, 8.05, 4.05), goal = at(g, 5.45, 4.05);
  auto legacy = g; legacy.cost_interpretation = CostInterpretation::ObstacleSeeds;
  EXPECT_FALSE(plan(legacy, start, goal, o).success);
  auto physical = g;
  for (auto & cost : physical.costs) {if (cost == 253) {cost = 0;}}
  for (bool optimize : {false, true}) {
    o.optimize = optimize;
    const auto result = plan(g, start, goal, o);
    verify(g, start, goal, o, result);
    // Independent continuous segment-to-square oracle checks the whole swept circle.
    EXPECT_GT(benchmark_geometry::clearance(physical, result.path, o.radius + o.clearance), 0.0);
  }
  EXPECT_EQ(g.costs, legacy.costs);  // interpretation never clears the caller's grid
}

TEST(Nav2CostSemantics, InscribedCellsRemainForbiddenEvenWithoutLethalSeeds)
{
  auto g = empty_grid(); auto o = test_options(); o.optimize = false;
  g.cost_interpretation = CostInterpretation::Nav2Master;
  for (int y = 0; y < g.height; ++y) {g.costs[y * g.width + 50] = 253;}
  EXPECT_FALSE(collision_free(g, {at(g, 5.05, 4.05)}, o));
  EXPECT_FALSE(collision_free(g, {at(g, 4.05, 4.05), at(g, 6.05, 4.05)}, o));
  EXPECT_FALSE(plan(g, at(g, 4.05, 4.05), at(g, 6.05, 4.05), o).success);
  // 253 is a centre exclusion, not an inferred physical obstacle to dilate again.
  EXPECT_TRUE(collision_free(g, {at(g, 5.25, 4.05)}, o));
}

TEST(Nav2CostSemantics, LethalUnknownAndBoundaryStillReserveTheFullRadius)
{
  auto o = test_options();
  for (uint8_t cost : {254, 255}) {
    auto g = empty_grid(); g.cost_interpretation = CostInterpretation::Nav2Master;
    g.costs[40 * g.width + 50] = cost;
    EXPECT_FALSE(collision_free(g, {at(g, 5.25, 4.05)}, o));
    EXPECT_FALSE(collision_free(g, {at(g, 4.05, 4.05), at(g, 6.05, 4.05)}, o));
    EXPECT_TRUE(collision_free(g, {at(g, 5.55, 4.05)}, o));
    EXPECT_FALSE(collision_free(g, {at(g, .25, 4.05)}, o));
    EXPECT_FALSE(collision_free(g, {at(g, -.01, 4.05)}, o));
  }
}

TEST(Nav2CostSemantics, PreparedSnapshotRetainsInterpretationAndIsImmutable)
{
  auto g = nav2_wall_grid(); auto o = test_options(); o.optimize = false;
  const auto start = at(g, 8.05, 4.05), goal = at(g, 5.45, 4.05);
  const auto prepared = prepare_grid(g, o);
  const auto raw = plan(g, start, goal, o), cached = plan_prepared(prepared, start, goal, o);
  ASSERT_TRUE(raw.success); ASSERT_TRUE(cached.success);
  ASSERT_EQ(raw.path.size(), cached.path.size());
  for (size_t i = 0; i < raw.path.size(); ++i) {
    EXPECT_DOUBLE_EQ(raw.path[i].x, cached.path[i].x);
    EXPECT_DOUBLE_EQ(raw.path[i].y, cached.path[i].y);
  }
  EXPECT_TRUE(collision_free_prepared(prepared, cached.path));
  g.cost_interpretation = CostInterpretation::ObstacleSeeds;
  g.costs.assign(g.costs.size(), 254);
  EXPECT_TRUE(plan_prepared(prepared, start, goal, o).success);
  EXPECT_FALSE(plan(g, start, goal, o).success);
  o.radius += .1;
  EXPECT_FALSE(plan_prepared(prepared, start, goal, o).success);
}

TEST(Nav2CostSemantics, EndpointReasonsDistinguishOutsideBlockedAndFree)
{
  auto g = nav2_wall_grid(); auto o = test_options(); o.optimize = false;
  const auto free = at(g, 8.05, 4.05), blocked = at(g, 5.05, 4.05);
  auto result = plan(g, free, blocked, o);
  EXPECT_FALSE(result.success); EXPECT_TRUE(result.path.empty());
  EXPECT_NE(result.reason.find("start=free@cost=0, goal=blocked@cost=254"), std::string::npos);
  result = plan(g, blocked, free, o);
  EXPECT_NE(result.reason.find("start=blocked@cost=254, goal=free@cost=0"), std::string::npos);
  result = plan(g, free, at(g, -1, 4.05), o);
  EXPECT_NE(result.reason.find("goal=outside_map"), std::string::npos);
  g.cost_interpretation = static_cast<CostInterpretation>(42);
  result = plan(g, free, free, o);
  EXPECT_FALSE(result.success); EXPECT_TRUE(result.path.empty());
  EXPECT_EQ(result.reason, "unsupported cost interpretation");
}
