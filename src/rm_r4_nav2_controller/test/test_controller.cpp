#include <gtest/gtest.h>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_loader.hpp>
#include <thread>
#include <limits>
#include "rm_r4_nav2_controller/controller.hpp"
#include "fixtures.hpp"
namespace nc = rm_r4_nav2_controller;
namespace r4 = rm_r4_prediction_consumption;
namespace
{
r4::FollowInput input(uint64_t sequence = 1)
{
  auto b = r4_test::body(); const int64_t t = r4_test::source + sequence * 50000000;
  auto e = r4_test::envelope(t); e.sequence = sequence + 2; e.prediction.tracks.clear();
  e.prediction.total_track_count = 0; e.tracks.clear();
  auto snap = r4::PredictionSnapshot::freeze(e, t, "map", b);
  auto route = r4::PreparedCorridor::prepare(r4_test::path(), r4_test::grid(), b, 4);
  return {snap, route, b, {{-.3, -.5}, {.5, .5}, {.8, .8}, .4, .5}, {"original-host", "action-1", "base_link", 1, sequence},
    {{0, 0}, {0, 0}, {0, 0}, 0, 0, 0, t, t, t, t, "map", "base_link"}, 1., r4::FollowClock::now(), false};
}
geometry_msgs::msg::PoseStamped pose(const r4::FollowInput & in)
{
  geometry_msgs::msg::PoseStamped p; p.header.frame_id = in.state.frame;
  p.header.stamp = r4_test::stamp(in.state.pose_stamp_ns); p.pose.position.x = in.state.position.x;
  p.pose.position.y = in.state.position.y; p.pose.orientation.w = 1.; return p;
}
geometry_msgs::msg::Twist velocity(const r4::FollowInput & in)
{
  geometry_msgs::msg::Twist v; v.linear.x = in.state.measured_body_velocity.x;
  v.linear.y = in.state.measured_body_velocity.y; v.angular.z = in.state.measured_yaw_rate; return v;
}
nc::CycleToken token(const nc::CycleAdapter & c, const r4::FollowInput & in)
{return {in.identity, c.plan_revision(), c.speed_limit_revision(), c.lifecycle_revision()};}
class ControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("r4_controller_finite_check");
    controller.configure(parent, "R4", nullptr, nullptr); controller.setPlan(r4_test::path()); controller.activate();
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent;
  nc::R4Controller controller;
};
}
TEST_F(ControllerTest, InstalledPluginlibLoadsStandardControllerAndPrivateAdapter)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto base = loader.createSharedInstance("rm_r4_nav2_controller/R4Controller");
  auto * adapter = dynamic_cast<nc::CycleAdapter *>(base.get()); ASSERT_NE(adapter, nullptr);
  base->configure(parent, "loaded-R4", nullptr, nullptr); base->setPlan(r4_test::path()); base->activate();
  auto in = input(); auto t = token(*adapter, in); ASSERT_TRUE(adapter->bind_cycle({t, in}));
  auto out = base->computeVelocityCommands(pose(in), velocity(in), nullptr);
  auto r = adapter->take_result(t); ASSERT_TRUE(r); ASSERT_TRUE(r->result.proposal) << r->result.reason;
  EXPECT_EQ(out.header.frame_id, "base_link"); EXPECT_GT(out.twist.linear.x, 0.); base->deactivate(); base->cleanup();
}
TEST_F(ControllerTest, NativeReturnAndTypedProposalMatchExistingFollow)
{
  auto in = input(); auto t = token(controller, in); r4::FollowSolver direct;
  auto reference = direct.solve(in); ASSERT_TRUE(reference.proposal) << reference.reason;
  ASSERT_TRUE(controller.bind_cycle({t, in})); const auto out = controller.computeVelocityCommands(pose(in), velocity(in), nullptr);
  auto r = controller.take_result(t); ASSERT_TRUE(r); ASSERT_TRUE(r->result.proposal) << r->result.reason;
  EXPECT_NEAR(out.twist.linear.x, reference.proposal->body_velocity.x, 1e-12);
  EXPECT_DOUBLE_EQ(out.twist.linear.x, r->result.proposal->body_velocity.x);
  EXPECT_DOUBLE_EQ(out.twist.linear.y, r->result.proposal->body_velocity.y);
  EXPECT_EQ(r->result.proposal->input_digest, reference.proposal->input_digest);
  EXPECT_EQ(r->result.proposal->acquired, in.acquired);
  EXPECT_EQ(r->result.proposal->source_deadline, in.acquired + std::chrono::milliseconds(75));
  EXPECT_EQ(out.header.stamp, r4_test::stamp(in.prediction.epoch_ns()));
}
TEST_F(ControllerTest, StandardHostWithoutTypedContextCannotCompute)
{
  auto in = input(); EXPECT_THROW(controller.computeVelocityCommands(pose(in), velocity(in), nullptr), nav2_core::PlannerException);
  EXPECT_FALSE(controller.take_result(token(controller, in)));
}
TEST_F(ControllerTest, ContextAndResultAreSingleUseAndWrongTokenClears)
{
  auto in = input(); auto t = token(controller, in); ASSERT_TRUE(controller.bind_cycle({t, in}));
  controller.computeVelocityCommands(pose(in), velocity(in), nullptr);
  auto wrong = t; ++wrong.identity.cycle_sequence; EXPECT_FALSE(controller.take_result(wrong)); EXPECT_FALSE(controller.take_result(t));
  auto next = input(2); auto nt = token(controller, next); ASSERT_TRUE(controller.bind_cycle({nt, next}));
  controller.computeVelocityCommands(pose(next), velocity(next), nullptr); ASSERT_TRUE(controller.take_result(nt));
  EXPECT_FALSE(controller.take_result(nt));
  EXPECT_THROW(controller.computeVelocityCommands(pose(next), velocity(next), nullptr), nav2_core::PlannerException);
}
TEST_F(ControllerTest, ExactPathMetadataAndPointsBindPreparedCorridor)
{
  for (int change = 0; change < 4; ++change) {
    auto p = r4_test::path(); if (change == 0) {p.poses[0].pose.position.x += .01;}
    if (change == 1) {p.header.stamp.nanosec = 1;} if (change == 2) {p.poses[0].header.frame_id.clear();}
    if (change == 3) {p.poses[0].pose.orientation.z = .01;}
    controller.setPlan(p); auto in = input(); EXPECT_FALSE(controller.bind_cycle({token(controller, in), in}));
  }
  controller.setPlan(r4_test::path()); auto in = input(); EXPECT_TRUE(controller.bind_cycle({token(controller, in), in}));
}
TEST_F(ControllerTest, PlanSpeedLifecycleRevisionsRejectOldContext)
{
  for (int change = 0; change < 3; ++change) {
    auto in = input(); auto t = token(controller, in);
    if (change == 0) {controller.setPlan(r4_test::path());}
    if (change == 1) {controller.setSpeedLimit(0., false);}
    if (change == 2) {controller.deactivate(); controller.activate();}
    EXPECT_FALSE(controller.bind_cycle({t, in}));
    EXPECT_TRUE(controller.bind_cycle({token(controller, in), in})); controller.deactivate(); controller.activate();
  }
}
TEST_F(ControllerTest, StandardPoseStampFrameQuaternionAndBodyVelocityMustMatch)
{
  for (int change = 0; change < 7; ++change) {
    auto in = input(); auto p = pose(in); auto v = velocity(in); auto t = token(controller, in);
    ASSERT_TRUE(controller.bind_cycle({t, in}));
    if (change == 0) {p.header.frame_id = "odom";} if (change == 1) {++p.header.stamp.nanosec;}
    if (change == 2) {p.pose.position.x += .001;} if (change == 3) {p.pose.orientation.w = .99;}
    if (change == 4) {p.pose.orientation.x = .001;} if (change == 5) {v.linear.y = .001;}
    if (change == 6) {v.angular.x = .001;}
    EXPECT_THROW(controller.computeVelocityCommands(p, v, nullptr), nav2_core::PlannerException); EXPECT_FALSE(controller.take_result(t));
  }
}
TEST_F(ControllerTest, NonzeroOrInvalidSpeedLimitIsUnavailableAndClearsPending)
{
  for (double limit : {.1, 50., -1., std::numeric_limits<double>::quiet_NaN()}) {
    controller.setSpeedLimit(0., false); auto in = input(); auto t = token(controller, in); ASSERT_TRUE(controller.bind_cycle({t, in}));
    controller.setSpeedLimit(limit, true); EXPECT_FALSE(controller.take_result(t));
    EXPECT_FALSE(controller.bind_cycle({token(controller, in), in}));
    EXPECT_THROW(controller.computeVelocityCommands(pose(in), velocity(in), nullptr), nav2_core::PlannerException);
  }
}
TEST_F(ControllerTest, ExpiredAcquisitionReturnsTypedFailureWithoutBrake)
{
  auto in = input(); in.acquired -= std::chrono::milliseconds(41); auto t = token(controller, in);
  ASSERT_TRUE(controller.bind_cycle({t, in}));
  EXPECT_THROW(controller.computeVelocityCommands(pose(in), velocity(in), nullptr), nav2_core::PlannerException);
  auto r = controller.take_result(t); ASSERT_TRUE(r); EXPECT_FALSE(r->result.proposal); EXPECT_EQ(r->result.reason, "cycle_deadline_before_assembly");
}
TEST_F(ControllerTest, OriginalLeaseCanExpireBeforeHostTakesResult)
{
  auto in = input(); auto t = token(controller, in); ASSERT_TRUE(controller.bind_cycle({t, in}));
  controller.computeVelocityCommands(pose(in), velocity(in), nullptr);
  std::this_thread::sleep_until(in.acquired + std::chrono::milliseconds(80));
  auto r = controller.take_result(t); ASSERT_TRUE(r); EXPECT_FALSE(r->result.proposal); EXPECT_EQ(r->result.reason, "source_expired_before_host_take");
}
TEST_F(ControllerTest, LifecycleInvalidatesHeldContextAndResult)
{
  auto in = input(); auto t = token(controller, in); ASSERT_TRUE(controller.bind_cycle({t, in})); controller.deactivate();
  EXPECT_THROW(controller.computeVelocityCommands(pose(in), velocity(in), nullptr), nav2_core::PlannerException);
  controller.activate(); auto next = input(2); auto nt = token(controller, next); ASSERT_TRUE(controller.bind_cycle({nt, next}));
  controller.computeVelocityCommands(pose(next), velocity(next), nullptr); controller.cleanup(); EXPECT_FALSE(controller.take_result(nt));
  EXPECT_THROW(controller.activate(), nav2_core::PlannerException);
}
