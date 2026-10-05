#include <gtest/gtest.h>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_loader.hpp>
#include <thread>
#include <limits>
#include "rm_r4_nav2_controller/controller.hpp"
#include "rm_r4_nav2_controller/proposal_bridge.hpp"
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
nc::ex::ExecutionFence fence(const r4::FollowInput & in)
{
  return {"original-authority", in.identity.host_instance, "R4", in.identity.execution_id,
    "original-smoother", in.identity.base_frame, "ROS-system", "clock-generation", "plan-policy",
    "actual-body", "actual-limits", in.identity.authority_epoch, true};
}
nc::SourcePublishStamp published(const r4::FollowInput & in)
{
  const auto now = r4::FollowClock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - in.acquired).count();
  return {now, in.prediction.epoch_ns() + elapsed, 0, "ROS-system", "clock-generation", true};
}
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
TEST_F(ControllerTest, SourceMapperKeepsOriginalAcquisitionAndSingleAtomicBudget)
{
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  auto native = controller.computeVelocityCommands(pose(in), velocity(in), nullptr); auto result = controller.take_result(t);
  auto stamp = published(in); auto mapped = nc::map_source_proposal(captured, result, native, f, stamp);
  ASSERT_TRUE(mapped.packet) << mapped.reason; const auto & p = *mapped.packet;
  EXPECT_EQ(p.acquired_epoch_ns, in.prediction.epoch_ns()); EXPECT_EQ(p.source_elapsed_ns + p.remaining_ns, 75000000);
  EXPECT_EQ(p.cycle_sequence, in.identity.cycle_sequence); EXPECT_DOUBLE_EQ(p.command.vx, native.twist.linear.x);
  ASSERT_TRUE(p.provenance); EXPECT_EQ(p.provenance->receipt_digest, in.prediction.receipt_digest());
  EXPECT_EQ(p.provenance->body_digest, in.body.digest());
  nc::ex::ProposalReceiptGate gate(f.owner_instance, "receiver-clock", f.authority_instance); ASSERT_TRUE(gate.observe_fence(f));
  nc::ex::TransportWitness w{f.owner_instance, "receiver-clock", f.clock_domain, f.clock_generation,
    stamp.epoch_ns + 2000000, 10000000000, 0, 0, 5000000, true};
  auto receipt = gate.receive(p, w); ASSERT_TRUE(receipt.proposal) << receipt.reason;
  ASSERT_TRUE(receipt.proposal->source().provenance);
  EXPECT_EQ(receipt.proposal->source().provenance->input_digest, p.provenance->input_digest);
  EXPECT_EQ(receipt.proposal->deadline_steady_ns(), w.receipt_steady_ns + p.remaining_ns - 2000000);
  EXPECT_FALSE(gate.receive(p, w).proposal);
}
TEST_F(ControllerTest, SourceMapperRejectsOldTokenFenceAndInputIdentity)
{
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  auto native = controller.computeVelocityCommands(pose(in), velocity(in), nullptr); auto result = controller.take_result(t); ASSERT_TRUE(result);
  for (int change = 0; change < 8; ++change) {
    auto bad = result; auto active = f;
    if (change == 0) {++bad->token.plan_revision;} if (change == 1) {++active.authority_epoch;}
    if (change == 2) {bad->result.proposal->input_digest = "other-cycle";}
    if (change == 3) {bad->result.proposal->identity.execution_id = "other-action";}
    if (change == 4) {bad->result.proposal->receipt_digest = "other-receipt";}
    if (change == 5) {bad->result.proposal->path_digest = "other-path";}
    if (change == 6) {bad->result.proposal->map_digest = "other-map";}
    if (change == 7) {bad->result.proposal->limits_digest = "other-limits";}
    EXPECT_FALSE(nc::map_source_proposal(captured, bad, native, active, published(in)).packet);
  }
}
TEST_F(ControllerTest, SourceMapperRejectsNativeZeroFrameStampAndExtraAxes)
{
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  auto native = controller.computeVelocityCommands(pose(in), velocity(in), nullptr); auto result = controller.take_result(t);
  for (int change = 0; change < 5; ++change) {
    auto bad = native; if (change == 0) {bad.twist = geometry_msgs::msg::Twist{};}
    if (change == 1) {bad.header.frame_id = "map";} if (change == 2) {++bad.header.stamp.nanosec;}
    if (change == 3) {bad.twist.linear.z = .001;} if (change == 4) {bad.twist.angular.x = .001;}
    EXPECT_FALSE(nc::map_source_proposal(captured, result, bad, f, published(in)).packet);
  }
}
TEST_F(ControllerTest, SourceMapperRejectsDeadlineRebasingAndOverBudgetResult)
{
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  auto native = controller.computeVelocityCommands(pose(in), velocity(in), nullptr); auto result = controller.take_result(t); ASSERT_TRUE(result);
  for (int change = 0; change < 5; ++change) {
    auto bad = result;
    if (change == 0) {bad->result.proposal->source_deadline += std::chrono::milliseconds(1);}
    if (change == 1) {bad->result.elapsed_seconds = .04;} if (change == 2) {bad->result.solver_seconds = .0150001;}
    if (change == 3) {bad->result.iterations = 401;} if (change == 4) {bad->result.minimum_constraint_slack = -.000011;}
    EXPECT_FALSE(nc::map_source_proposal(captured, bad, native, f, published(in)).packet);
  }
}
TEST_F(ControllerTest, DelayedPublicationConsumesLeaseAndExpiryCannotRenew)
{
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  auto native = controller.computeVelocityCommands(pose(in), velocity(in), nullptr); auto result = controller.take_result(t);
  std::this_thread::sleep_until(in.acquired + std::chrono::milliseconds(60));
  auto mapped = nc::map_source_proposal(captured, result, native, f, published(in)); ASSERT_TRUE(mapped.packet) << mapped.reason;
  EXPECT_LE(mapped.packet->remaining_ns, 15000000); EXPECT_GT(mapped.packet->remaining_ns, 0);
  auto expired = published(in); expired.steady = in.acquired + std::chrono::milliseconds(75);
  expired.epoch_ns = in.prediction.epoch_ns() + 75000000;
  EXPECT_FALSE(nc::map_source_proposal(captured, result, native, f, expired).packet);
}
TEST_F(ControllerTest, SourceClockUnknownResetDeltaAndPreAcquisitionReject)
{
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  auto native = controller.computeVelocityCommands(pose(in), velocity(in), nullptr); auto result = controller.take_result(t);
  for (int change = 0; change < 5; ++change) {
    auto bad = published(in); if (change == 0) {bad.verified = false;} if (change == 1) {bad.clock_generation = "reset";}
    if (change == 2) {++bad.epoch_ns;} if (change == 3) {bad.steady = in.acquired - std::chrono::nanoseconds(1);}
    if (change == 4) {bad.clock_drift_bound_ns = 1000001;}
    EXPECT_FALSE(nc::map_source_proposal(captured, result, native, f, bad).packet);
  }
}
TEST_F(ControllerTest, CaptureDoesNotIssueAuthorityOrReplaceOriginalAcquisition)
{
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto wrong = f; wrong.active = false; EXPECT_THROW(nc::SourceContext::capture({t, in}, wrong), r4::ContractError);
  wrong = f; wrong.producer_instance = "other-host"; EXPECT_THROW(nc::SourceContext::capture({t, in}, wrong), r4::ContractError);
  auto bad = t; ++bad.identity.cycle_sequence; EXPECT_THROW(nc::SourceContext::capture({bad, in}, f), r4::ContractError);
  in.acquired = r4::FollowClock::now() + std::chrono::seconds(1);
  EXPECT_THROW(nc::SourceContext::capture({t, in}, f), r4::ContractError);
}
TEST_F(ControllerTest, TypedFailureCannotTurnNativeHostZeroIntoNormalProposal)
{
  auto in = input(); in.acquired -= std::chrono::milliseconds(41); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  EXPECT_THROW(controller.computeVelocityCommands(pose(in), velocity(in), nullptr), nav2_core::PlannerException);
  auto result = controller.take_result(t); ASSERT_TRUE(result); EXPECT_FALSE(result->result.proposal);
  EXPECT_FALSE(nc::map_source_proposal(captured, result, geometry_msgs::msg::TwistStamped{}, f, published(in)).packet);
}
TEST_F(ControllerTest, StandardToOriginalReceiptAndCurrentAdmissionUsesSameActualCandidate)
{
  namespace ex = nc::ex;
  auto in = input(); auto t = token(controller, in); auto f = fence(in);
  auto captured = nc::SourceContext::capture({t, in}, f); ASSERT_TRUE(controller.bind_cycle({t, in}));
  auto native = controller.computeVelocityCommands(pose(in), velocity(in), nullptr); auto result = controller.take_result(t);
  auto stamp = published(in); auto mapped = nc::map_source_proposal(captured, result, native, f, stamp); ASSERT_TRUE(mapped.packet) << mapped.reason;
  ex::ProposalReceiptGate gate(f.owner_instance, "receiver-clock", f.authority_instance); ASSERT_TRUE(gate.observe_fence(f));
  ex::TransportWitness w{f.owner_instance, "receiver-clock", f.clock_domain, f.clock_generation,
    stamp.epoch_ns + 2000000, 10000000000, 0, 0, 5000000, true};
  auto receipt = gate.receive(*mapped.packet, w); ASSERT_TRUE(receipt.proposal) << receipt.reason;
  const int64_t e = w.receipt_epoch_ns;
  ex::RawCurrentGrid grid{80, 80, .05, -2, -2, std::vector<uint8_t>(6400, 0), {"odom", "current-map", e, e, true}};
  ex::CurrentCommandInput current{ex::CurrentGridSnapshot(grid),
    {{{-.34, -.29}, {.34, -.29}, {.34, .29}, {-.34, .29}}, .02, .05, f.body_revision},
    {{-.3, -.5}, {.5, .5}, 1.2, 0, 0, 0, 0, f.limits_revision}, {0, 0, 0}, {0, 0, 0}, {.02, 0, 0},
    "odom", "base_link", "final-owner-candidate", e, e, e, e, 50000000};
  auto proof = ex::inspect_current_command(current); ASSERT_EQ(proof.status, ex::GeometryStatus::Certified) << proof.reason;
  const int64_t elapsed = std::ceil(proof.elapsed_seconds * 1e9);
  ex::SendContext send{f.owner_instance, "receiver-clock", current.candidate_identity, "current-map", f.body_revision,
    f.limits_revision, "odom", 1, current.candidate, e, w.receipt_steady_ns, w.receipt_steady_ns + elapsed, 1000000, true};
  auto applied = ex::admit_for_send(*receipt.proposal, gate, proof, send); ASSERT_TRUE(applied.command) << applied.reason;
  EXPECT_DOUBLE_EQ(applied.command->actual_command.vx, .02);
  EXPECT_DOUBLE_EQ(applied.command->source.command.vx, native.twist.linear.x);
  EXPECT_LE(applied.command->valid_until_steady_ns, receipt.proposal->deadline_steady_ns());
  EXPECT_EQ(applied.command->source.acquired_epoch_ns, in.prediction.epoch_ns());
  ASSERT_TRUE(applied.command->source.provenance);
  EXPECT_EQ(applied.command->source.provenance->input_digest, result->result.proposal->input_digest);
}
