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
  auto wait = d::check_command({0, 0, 0}, {0, 0, 0}, {0, 0, 0}, footprint, a,
                               lim, 0, {}, cfg, clear);
  EXPECT_TRUE(wait.pass);
  a.tracks[0].position.x = .8;
  auto measured = d::check_command({0, 0, 0}, {1, 0, 0}, {0, 0, 0}, footprint,
                                   a, lim, 0, {}, cfg, clear);
  EXPECT_FALSE(measured.pass); // Publishing zero cannot instantaneously erase
                               // measured momentum.
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
