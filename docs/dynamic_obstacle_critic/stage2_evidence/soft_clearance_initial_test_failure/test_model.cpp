#include "rm_dynamic_obstacle_critic/guard.hpp"
#include "rm_dynamic_obstacle_critic/static_map.hpp"
#include <gtest/gtest.h>
namespace d = rm_dynamic_obstacle_critic;
static std::vector<d::Point> footprint{
    {-.25, -.2}, {.25, -.2}, {.25, .2}, {-.25, .2}};
static d::Array obstacles(double x, double y, double vx, double vy) {
  d::Array a;
  a.schema = d::Array::SCHEMA;
  a.authority = d::Array::AUTHORITY_SHADOW_ONLY;
  a.header.frame_id = "map";
  a.header.stamp.sec = 10;
  a.processing_stamp = a.header.stamp;
  a.prediction_dt = .1;
  a.prediction_steps = 30;
  a.complete = true;
  a.total_track_count = 1;
  d::Obstacle t;
  t.track_id = 1;
  t.state = t.STATE_CONFIRMED;
  t.position.x = x;
  t.position.y = y;
  t.velocity.x = vx;
  t.velocity.y = vy;
  t.size.x = .2;
  t.size.y = .2;
  t.last_observation_stamp = a.header.stamp;
  a.tracks.push_back(t);
  return a;
}
static std::vector<d::Pose> straight(double speed) {
  std::vector<d::Pose> result;
  for (int k = 1; k <= 30; ++k)
    result.push_back({k * .1 * speed, 0, 0});
  return result;
}
TEST(CV, FutureConflictAndCrossing) {
  auto a = obstacles(1, -1, 0, 1);
  d::Limits lim;
  lim.minimum_radius = .15;
  d::CostParameters cost;
  auto moving = d::score(straight(1), footprint, a, lim, cost, 0, .1);
  auto wait = d::score(straight(0), footprint, a, lim, cost, 0, .1);
  EXPECT_GT(moving.cost, 1000);
  EXPECT_LT(moving.collision_time, 1.2);
  EXPECT_LT(wait.cost, 1);
  EXPECT_GT(wait.minimum_clearance, .5);
  EXPECT_EQ(d::validate(a, 10, lim), "");
}
TEST(CV, CurrentOccupancyFutureRelease) {
  auto a = obstacles(1, 0, 0, 1);
  d::Limits lim;
  lim.minimum_radius = .15;
  d::CostParameters cost;
  auto release = d::score(straight(.5), footprint, a, lim, cost, 0, .1);
  a.tracks[0].velocity.y = 0;
  auto frozen = d::score(straight(.5), footprint, a, lim, cost, 0, .1);
  EXPECT_LT(release.cost, 1);
  EXPECT_GT(frozen.cost, 1000);
}
TEST(CV, StationaryMatchesGeometryAndAll30Steps) {
  auto a = obstacles(2.6, 0, 0, 0);
  d::Limits lim;
  lim.minimum_radius = .15;
  d::CostParameters cost;
  auto risk = d::score(straight(1), footprint, a, lim, cost, 0, .1);
  EXPECT_GT(risk.collision_time, 2);
  EXPECT_LT(risk.collision_time, 3);
  EXPECT_GT(risk.cost, 1000);
  EXPECT_NEAR(d::predict(a.tracks[0], .3, 2).x, 2.6, 1e-12);
}
TEST(CV, SourceAgeAndRotatedFrame) {
  auto a = obstacles(1, 0, 1, 0);
  auto p = d::predict(a.tracks[0], .2, 1, {2, 3, 1.5707963267948966});
  EXPECT_NEAR(p.x, 2, 1e-10);
  EXPECT_NEAR(p.y, 5.2, 1e-10);
}
TEST(Cost, ContinuousMarginsAndContact) {
  d::CostParameters p;
  for (double x : {p.influence_distance, p.safety_margin, 0.0})
    EXPECT_NEAR(d::penalty(x - 1e-8, p), d::penalty(x + 1e-8, p), .03);
  EXPECT_EQ(d::penalty(.5, p), 0);
  EXPECT_GT(d::penalty(0, p), 9999);
}
TEST(Input, StaleInvalidIncompleteCoastAndJump) {
  d::Limits l;
  auto a = obstacles(1, 0, 0, 0);
  EXPECT_NE(d::validate(a, 10.41, l), "");
  EXPECT_NE(d::validate(a, 9.99, l), "");
  a.complete = false;
  EXPECT_NE(d::validate(a, 10, l), "");
  a.complete = true;
  a.tracks[0].velocity.x = NAN;
  EXPECT_NE(d::validate(a, 10, l), "");
  a.tracks[0].velocity.x = 0;
  a.tracks[0].state = d::Obstacle::STATE_COASTING;
  a.tracks[0].last_observation_stamp.sec = 9;
  EXPECT_NE(d::validate(a, 10, l), "");
  a.tracks[0].last_observation_stamp = a.header.stamp;
  d::InputCache cache;
  cache.receive(std::make_shared<d::Array>(a), l);
  a.header.stamp.nanosec = 100000000;
  a.tracks[0].last_observation_stamp = a.header.stamp;
  a.tracks[0].position.x = 5;
  cache.receive(std::make_shared<d::Array>(a), l);
  std::string reason;
  auto next = cache.get(reason);
  EXPECT_EQ(reason, "track_jump");
  EXPECT_EQ(next->tracks[0].position.x, 5);
}
TEST(Guard, PassBrakeAndMeasuredStoppingTail) {
  auto a = obstacles(1, 0, 0, 0);
  d::Limits lim;
  lim.minimum_radius = .15;
  d::GuardParameters cfg;
  auto clear = [](const auto &, double) { return true; };
  auto drive = d::check_command({0, 0, 0}, {0, 0, 0}, {1, 0, 0}, footprint, a,
                                lim, 0, {}, cfg, clear);
  EXPECT_FALSE(drive.pass);
  EXPECT_EQ(drive.reason, "dynamic_collision");
  EXPECT_EQ(drive.collision_branch, 1);
  EXPECT_GT(drive.collision_pose.x, 0);
  auto wait = d::check_command({0, 0, 0}, {0, 0, 0}, {0, 0, 0}, footprint, a,
                               lim, 0, {}, cfg, clear);
  EXPECT_TRUE(wait.pass);
  a.tracks[0].position.x = .8;
  auto measured = d::check_command({0, 0, 0}, {1, 0, 0}, {0, 0, 0}, footprint,
                                   a, lim, 0, {}, cfg, clear);
  EXPECT_FALSE(measured.pass); // Publishing zero cannot instantaneously erase
                               // measured momentum.
  EXPECT_EQ(measured.collision_branch, 0);
  cfg.horizon = 1e9;
  auto budget = d::check_command({0, 0, 0}, {0, 0, 0}, {0, 0, 0}, footprint, a,
                                 lim, 0, {}, cfg, clear);
  EXPECT_FALSE(budget.pass);
  EXPECT_EQ(budget.reason, "simulation_budget_exceeded");
}
TEST(Guard, StaticInteriorUnknownAndBounds) {
  nav2_msgs::msg::Costmap map;
  map.metadata.size_x = 40;
  map.metadata.size_y = 40;
  map.metadata.resolution = .1;
  map.metadata.origin.position.x = -2;
  map.metadata.origin.position.y = -2;
  map.metadata.origin.orientation.w = 1;
  map.data.assign(1600, 0);
  EXPECT_TRUE(d::static_map_clear(map, footprint, 0));
  map.data[20 * 40 + 20] = 203;
  EXPECT_FALSE(d::static_map_clear(map, footprint, 0));
  map.data[20 * 40 + 20] = 255;
  EXPECT_FALSE(d::static_map_clear(map, footprint, 0));
  EXPECT_FALSE(d::static_map_clear(map, d::transform(footprint, 3, 0, 0), 0));
  map.metadata.origin.orientation.w = NAN;
  EXPECT_FALSE(d::static_map_clear(map, footprint, 0));
}
TEST(Geometry, ConvexValidationAndRotation) {
  EXPECT_NO_THROW(d::validate_footprint(footprint));
  EXPECT_THROW(
      d::validate_footprint({{0, 0}, {1, 0}, {.5, .1}, {1, 1}, {0, 1}}),
      std::invalid_argument);
  auto p = d::advance({0, 0, 0}, {1, 0, 1}, 1.5707963267948966);
  EXPECT_NEAR(p.x, 1, 1e-9);
  EXPECT_NEAR(p.y, 1, 1e-9);
}

TEST(Guard, StaticWitnessInRotatedMapAndFutureCommandBranch) {
  nav2_msgs::msg::Costmap map;
  map.metadata.size_x = 40;
  map.metadata.size_y = 40;
  map.metadata.resolution = .1;
  map.metadata.origin.position.x = 2;
  map.metadata.origin.position.y = -2;
  map.metadata.origin.orientation.z = std::sqrt(.5);
  map.metadata.origin.orientation.w = std::sqrt(.5);
  map.data.assign(1600, 0);
  map.data[20 * 40 + 20] = 203;
  auto check = d::check_static_map(map, footprint, 0);
  EXPECT_FALSE(check.clear);
  EXPECT_EQ(check.reason, "occupied_cell");
  EXPECT_EQ(check.cell_x, 20);
  EXPECT_EQ(check.cell_y, 20);
  EXPECT_EQ(check.cost, 203u);
  EXPECT_NEAR(check.cell_center.x, -.05, 1e-6);
  EXPECT_NEAR(check.cell_center.y, .05, 1e-6);
  EXPECT_EQ(check.distance, 0);
  map.data[20 * 40 + 20] = 255;
  EXPECT_EQ(d::check_static_map(map, footprint, 0).reason, "unknown_cell");
  EXPECT_EQ(
      d::check_static_map(map, d::transform(footprint, 10, 0, 0), 0).reason,
      "outside_map");
  auto a = obstacles(10, 10, 0, 0);
  auto guard = d::check_command(
      {0, 0, 0}, {}, {.8, 0, 0}, footprint, a, {}, 0, {}, {},
      [](const auto &poly, double) { return poly.front().x < .15; });
  EXPECT_FALSE(guard.pass);
  EXPECT_EQ(guard.reason, "static_collision_or_unknown");
  EXPECT_EQ(guard.collision_branch, 1);
  EXPECT_GT(guard.collision_time, .4);
  EXPECT_GT(guard.collision_pose.x, .4);
}

TEST(Geometry, SeparatingAxisBroadphaseNeverSkipsNearRotatedPolygons) {
  size_t distant = 0, near = 0;
  for (size_t i = 0; i < 1000; ++i) {
    const double a = i * .037;
    auto poly = d::transform(footprint, .6 * std::sin(a), .5 * std::cos(a), a);
    const d::Box box{.1, -.1, .15, -.05};
    const double reserve = (i % 5) * .03;
    const auto exact = d::polygon_box_distance(poly, box);
    const bool possible =
        d::polygon_box_may_be_within(poly, box, reserve + 1e-9);
    if (exact <= reserve + 1e-9) {
      ++near;
      EXPECT_TRUE(possible) << i;
    }
    distant += !possible;
  }
  EXPECT_GT(near, 0u);
  EXPECT_GT(distant, 0u);
  const d::Box touch{.25, -.1, .3, .1};
  EXPECT_TRUE(d::polygon_box_may_be_within(footprint, touch, 1e-9));
}

TEST(Guard, CachedStaticMeasuredBranchMatchesOriginalUnion) {
  d::Array empty;
  const auto clear = [](const auto &poly, double reserve) {
    return d::polygon_box_distance(poly, {.7, -.1, .75, .15}) > reserve + 1e-9;
  };
  for (d::Velocity measured :
       {d::Velocity{.25, .1, .2}, d::Velocity{.9, 0, 0}}) {
    const auto cached =
        d::check_command({}, measured, {}, footprint, empty, {}, 0, {}, {},
                         clear, d::GuardPaths::MeasuredOnly);
    for (size_t i = 0; i < 100; ++i) {
      const double a = i * .13;
      d::Velocity command{.8 * std::cos(a), .5 * std::sin(a),
                          1.2 * std::sin(a)};
      const auto original = d::check_command({}, measured, command, footprint,
                                             empty, {}, 0, {}, {}, clear);
      const auto proposed =
          d::check_command({}, {}, command, footprint, empty, {}, 0, {}, {},
                           clear, d::GuardPaths::ProposedOnly);
      EXPECT_EQ(original.pass, cached.pass && proposed.pass) << i;
    }
  }
}

TEST(Frame, BoundedWorldCorrectionAndNoMovingFrameInput) {
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf(clock);
  geometry_msgs::msg::TransformStamped tr;
  tr.header.frame_id = "odom";
  tr.child_frame_id = "map";
  tr.header.stamp.sec = 20;
  tr.transform.rotation.w = 1;
  tr.transform.translation.x = 2;
  ASSERT_TRUE(tf.setTransform(tr, "test", false));
  const rclcpp::Time now(20, 0, RCL_ROS_TIME);
  EXPECT_NEAR(d::frame_transform(tf, "odom", "map", now).x, 2, 1e-9);
  tf.clear();
  tr.header.stamp.sec = 19;
  ASSERT_TRUE(tf.setTransform(tr, "test", false));
  EXPECT_THROW(d::frame_transform(tf, "odom", "map", now),
               std::invalid_argument);
  tf.clear();
  tr.header.stamp.sec = 20;
  tr.transform.rotation.x = .1;
  tr.transform.rotation.w = std::sqrt(.99);
  ASSERT_TRUE(tf.setTransform(tr, "test", false));
  EXPECT_THROW(d::frame_transform(tf, "odom", "map", now),
               std::invalid_argument);
  auto a = obstacles(1, 0, 0, 0);
  a.header.frame_id = "base_link";
  EXPECT_NE(d::validate(a, 10, d::Limits{}), "");
}

TEST(Geometry, OptionalNearestMapQueryPreservesFirstWitnessAndExactDistance) {
  nav2_msgs::msg::Costmap map;
  map.metadata.size_x = 40;
  map.metadata.size_y = 40;
  map.metadata.resolution = .1;
  map.metadata.origin.position.x = -2;
  map.metadata.origin.position.y = -2;
  map.metadata.origin.orientation.w = 1;
  map.data.assign(1600, 0);
  map.data[20 * 40 + 24] = 203; // Earlier, farther cell: x=.4.
  map.data[21 * 40 + 23] = 255; // Later, nearer cell: x=.3.
  auto first = d::check_static_map(map, footprint, .2);
  auto nearest = d::check_static_map(map, footprint, .2, 203,
                                     d::StaticMapMode::NearestWithinReserve);
  EXPECT_EQ(first.cell_x, 24);
  EXPECT_EQ(first.cell_y, 20);
  EXPECT_EQ(nearest.cell_x, 23);
  EXPECT_EQ(nearest.cell_y, 21);
  EXPECT_EQ(nearest.reason, "unknown_cell");
  EXPECT_NEAR(nearest.distance, .05, 1e-6);
  EXPECT_FALSE(first.clear);
  EXPECT_FALSE(nearest.clear);
  map.data.assign(1600, 0);
  auto boundary =
      d::check_static_map(map, d::transform(footprint, 1.7, 0, 0), .2, 203,
                          d::StaticMapMode::NearestWithinReserve);
  EXPECT_EQ(boundary.reason, "outside_map");
  EXPECT_NEAR(boundary.distance, .05, 1e-6);
  auto large = d::check_static_map(map, footprint, 1e300, 203,
                                   d::StaticMapMode::NearestWithinReserve);
  EXPECT_FALSE(large.clear);
  EXPECT_NEAR(large.distance, 1.75, 1e-6);
  // Rotating the entire world/map leaves the local distance invariant.
  map.metadata.origin.position.x = 2;
  map.metadata.origin.position.y = -2;
  map.metadata.origin.orientation.z = std::sqrt(.5);
  map.metadata.origin.orientation.w = std::sqrt(.5);
  map.data[21 * 40 + 23] = 255;
  auto rotated =
      d::check_static_map(map, d::transform(footprint, 0, 0, M_PI_2), .2, 203,
                          d::StaticMapMode::NearestWithinReserve);
  EXPECT_NEAR(rotated.distance, .05, 1e-6);
}

TEST(Geometry, BoundedNearestQueryMatchesFullGridAndOriginalHardPredicate) {
  nav2_msgs::msg::Costmap map;
  map.metadata.size_x = 40;
  map.metadata.size_y = 40;
  map.metadata.resolution = .1;
  map.metadata.origin.position.x = -2;
  map.metadata.origin.position.y = -2;
  map.metadata.origin.orientation.w = 1;
  map.data.assign(1600, 0);
  for (size_t i = 0; i < 1600; i += 31)
    map.data[i] = i % 2 ? 203 : 255;
  for (size_t i = 0; i < 500; ++i) {
    auto poly = d::transform(footprint, 1.9 * std::sin(i * .11),
                             1.9 * std::cos(i * .17), i * .03);
    auto local = d::transform(poly, 2, 2, 0);
    const double extent = 40 * static_cast<double>(map.metadata.resolution);
    double exact = INFINITY;
    for (auto p : local)
      exact = std::min({exact, p.x, p.y, extent - p.x, extent - p.y});
    exact = std::max(0., exact);
    for (size_t y = 0; y < 40; ++y)
      for (size_t x = 0; x < 40; ++x)
        if (map.data[y * 40 + x] >= 203) {
          const double r = map.metadata.resolution;
          exact = std::min(
              exact, d::polygon_box_distance(
                         local, {x * r, y * r, (x + 1) * r, (y + 1) * r}));
        }
    const auto near = d::check_static_map(
        map, poly, .13, 203, d::StaticMapMode::NearestWithinReserve);
    if (exact <= .13 + 1e-9)
      EXPECT_NEAR(near.distance, exact, 1e-10) << i;
    else
      EXPECT_TRUE(near.clear) << i;
    const bool original = d::static_map_clear(map, poly, .02);
    const bool converted = near.reason == "outside_map"
                               ? d::static_map_clear(map, poly, .02)
                               : near.clear || near.distance > .02 + 1e-9;
    EXPECT_EQ(original, converted) << i;
  }
}
