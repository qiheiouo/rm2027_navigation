#include <algorithm>
#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "fixtures.hpp"
#include "rm_r4_prediction_consumption/follow.hpp"

using namespace r4_test;
namespace
{
FollowInput input(uint64_t cycle = 1, bool obstacle = false, double yaw = 0.)
{
  auto b = body(); b.yaw = yaw; auto e = envelope(source + cycle * 50000000); e.sequence = cycle + 2;
  if (!obstacle) {e.prediction.tracks.clear(); e.prediction.total_track_count = 0; e.tracks.clear();}
  const int64_t epoch = source + cycle * 50000000;
  auto prediction = PredictionSnapshot::freeze(e, epoch, "map", b);
  auto route = PreparedCorridor::prepare(path(), grid(), b, 4);
  // main old-car profile: vx [-.30,.50], vy +/- .50, accel/decel .80.
  return {prediction, route, b, {{-.3, -.5}, {.5, .5}, {.8, .8}, .4, .5},
    {"host-A", "action-A", "base_link", 1, cycle},
    {{0., 0.}, {0., 0.}, {0., 0.}, yaw, 0., 0., epoch, epoch, epoch, epoch, "map", "base_link"},
    1., FollowClock::now(), false};
}
void check_bounds(const FollowInput & in, const FollowProposal & p)
{
  Vec2 last = in.state.last_applied_body_velocity;
  for (size_t k = 0; k < p.controls.size(); ++k) {
    const auto c = p.controls[k]; const double dt = k == 0 ? .05 : .1;
    EXPECT_GE(c.body_velocity.x, in.limits.lower.x); EXPECT_LE(c.body_velocity.x, in.limits.upper.x);
    EXPECT_GE(c.body_velocity.y, in.limits.lower.y); EXPECT_LE(c.body_velocity.y, in.limits.upper.y);
    EXPECT_LE(std::abs(c.body_velocity.x - last.x), dt * in.limits.command_rate.x + 1e-12);
    EXPECT_LE(std::abs(c.body_velocity.y - last.y), dt * in.limits.command_rate.y + 1e-12);
    EXPECT_GE(c.progress_rate, 0.); EXPECT_LE(c.progress_rate, in.limits.progress_upper);
    last = c.body_velocity;
  }
  const auto box = in.route.local_bounds(in.state.position);
  for (const auto s : p.stages) {
    EXPECT_GE(s.position.x, box.xmin); EXPECT_LE(s.position.x, box.xmax);
    EXPECT_GE(s.position.y, box.ymin); EXPECT_LE(s.position.y, box.ymax);
    EXPECT_NEAR(s.yaw, in.state.yaw, 1e-12);
    EXPECT_GE(s.progress, -1e-5); EXPECT_LE(s.progress, in.route.arcs().back() + 1e-5);
  }
}
}

TEST(Follow, ActualLimitsFreeProgressAndNoForcedTerminalStop)
{
  auto in = input(); FollowSolver solver; const auto r = solver.solve(in);
  ASSERT_TRUE(r.proposal) << r.reason << " / " << r.solver_status;
  EXPECT_EQ(r.solver_status, "solved"); EXPECT_LE(r.iterations, 400);
  EXPECT_LT(r.elapsed_seconds, .04); EXPECT_LE(r.solver_seconds, .015);
  EXPECT_GT(r.proposal->body_velocity.x, 0.); EXPECT_NEAR(r.proposal->body_velocity.y, 0., 1e-7);
  EXPECT_GT(r.proposal->controls.back().body_velocity.x, .1);
  EXPECT_GT(r.proposal->stages.back().progress, in.progress);
  EXPECT_EQ(r.proposal->source_deadline - r.proposal->acquired, std::chrono::milliseconds(75));
  EXPECT_EQ(r.proposal->epoch_ns, in.prediction.epoch_ns());
  EXPECT_EQ(r.proposal->receipt_digest, in.prediction.receipt_digest());
  EXPECT_EQ(r.proposal->identity.cycle_sequence, in.identity.cycle_sequence);
  EXPECT_EQ(r.proposal->input_digest.size(), 64u); check_bounds(in, *r.proposal);
}
TEST(Follow, FixedYawRotatesBodyTargetsAndExactRollout)
{
  auto in = input(1, false, std::acos(-1.) / 2.); FollowSolver solver; const auto r = solver.solve(in);
  ASSERT_TRUE(r.proposal) << r.reason << " / " << r.solver_status;
  EXPECT_LT(r.proposal->body_velocity.y, 0.); EXPECT_NEAR(r.proposal->body_velocity.x, 0., 1e-7);
  double x = in.state.position.x, y = in.state.position.y, s = in.progress;
  for (size_t k = 1; k < r.proposal->stages.size(); ++k) {
    const auto c = r.proposal->controls[(k - 1) / 2];
    x += .05 * (std::cos(in.state.yaw) * c.body_velocity.x - std::sin(in.state.yaw) * c.body_velocity.y);
    y += .05 * (std::sin(in.state.yaw) * c.body_velocity.x + std::cos(in.state.yaw) * c.body_velocity.y);
    s += .05 * c.progress_rate;
    EXPECT_NEAR(r.proposal->stages[k].position.x, x, 1e-12);
    EXPECT_NEAR(r.proposal->stages[k].position.y, y, 1e-12);
    EXPECT_NEAR(r.proposal->stages[k].progress, s, 1e-12);
  }
  check_bounds(in, *r.proposal);
}
TEST(Follow, StartsRateBoundFromExplicitActuallyAppliedValue)
{
  auto in = input(); in.state.last_applied_body_velocity = {.2, -.1};
  FollowSolver solver; const auto r = solver.solve(in);
  ASSERT_TRUE(r.proposal) << r.reason << " / " << r.solver_status; check_bounds(in, *r.proposal);
  EXPECT_GT(r.proposal->body_velocity.x, .15);
}
TEST(Follow, FutureObservedOccupancyOnlyAddsSoftCost)
{
  auto in = input(1, true); auto e = envelope(in.prediction.epoch_ns());
  e.prediction.tracks[0].velocity.x = 0.; e.tracks[0].centroid_at_observation.x = .7; align(e);
  in.prediction = PredictionSnapshot::freeze(e, in.prediction.epoch_ns(), "map", in.body);
  FollowSolver solver; const auto r = solver.solve(in);
  ASSERT_TRUE(r.proposal) << r.reason << " / " << r.solver_status;
  EXPECT_GT(r.nominal_dynamic_cost, 0.); EXPECT_GT(r.solved_dynamic_cost, 0.);
  check_bounds(in, *r.proposal);
}
TEST(Follow, WarmShiftNeedsSameContextAndNewCycle)
{
  FollowSolver solver; auto first = solver.solve(input()); ASSERT_TRUE(first.proposal) << first.reason;
  auto second = solver.solve(input(2)); ASSERT_TRUE(second.proposal) << second.reason;
  EXPECT_TRUE(second.used_warm);
  auto changed = input(3); changed.identity.execution_id = "action-B";
  const auto third = solver.solve(changed); ASSERT_TRUE(third.proposal) << third.reason;
  EXPECT_FALSE(third.used_warm); EXPECT_NE(first.proposal->input_digest, third.proposal->input_digest);
}
TEST(Follow, ReceiptResetAndMapOrLimitsRevisionClearWarm)
{
  for (int change = 0; change < 3; ++change) {
    FollowSolver solver; ASSERT_TRUE(solver.solve(input()).proposal);
    auto next = input(2);
    if (change == 0) {next.reset_warm = true;}
    if (change == 1) {auto g = grid(); g.data[0] = -1; next.route = PreparedCorridor::prepare(path(), g, next.body, 4);}
    if (change == 2) {next.limits.cruise = .3;}
    const auto r = solver.solve(next); ASSERT_TRUE(r.proposal) << r.reason; EXPECT_FALSE(r.used_warm);
  }
}
TEST(Follow, DuplicateEpochFailureDoesNotRetainWarm)
{
  FollowSolver solver; auto in = input(); ASSERT_TRUE(solver.solve(in).proposal);
  in.acquired = FollowClock::now(); const auto stale = solver.solve(in);
  EXPECT_FALSE(stale.proposal); EXPECT_EQ(stale.reason, "nonincreasing_cycle");
  const auto fresh = solver.solve(input(2)); ASSERT_TRUE(fresh.proposal) << fresh.reason; EXPECT_FALSE(fresh.used_warm);
}
TEST(Follow, ExpiredOrFutureAcquisitionCannotRebaseLease)
{
  FollowSolver solver; auto old = input(); old.acquired -= std::chrono::milliseconds(41);
  auto r = solver.solve(old); EXPECT_FALSE(r.proposal); EXPECT_EQ(r.reason, "cycle_deadline_before_assembly");
  auto future = input(2); future.acquired += std::chrono::seconds(1);
  r = solver.solve(future); EXPECT_FALSE(r.proposal); EXPECT_EQ(r.reason, "invalid_acquisition");
}
TEST(Follow, MissingStaleFrameBodyAndRotationValuesAreUnavailable)
{
  for (int change = 0; change < 10; ++change) {
    auto in = input();
    if (change == 0) {in.state.pose_stamp_ns -= 100000001;}
    if (change == 1) {in.state.tf_stamp_ns -= 1;}
    if (change == 2) {in.body.padding += .01;}
    if (change == 3) {in.state.last_applied_yaw_rate = .2;}
    if (change == 4) {in.state.measured_yaw_rate = .2;}
    if (change == 5) {in.state.position.x = std::numeric_limits<double>::quiet_NaN();}
    if (change == 6) {in.identity.base_frame.clear();}
    if (change == 7) {in.state.pose_stamp_ns += 1; in.state.tf_stamp_ns = in.state.pose_stamp_ns;}
    if (change == 8) {in.state.frame = "odom";}
    if (change == 9) {in.state.body_frame = "base_footprint";}
    FollowSolver solver; const auto r = solver.solve(in); EXPECT_FALSE(r.proposal); EXPECT_EQ(r.solver_status, "not_run");
  }
}
TEST(Follow, StaticInfeasibilityReturnsUnavailableWithoutBrakeProposal)
{
  auto in = input(); double outer = -1e10;
  // Use the outermost edge: local_bounds can select a different overlapping box.
  for (const auto b : in.route.centre_bounds()) {outer = std::max(outer, b.xmax);}
  in.state.position = {outer - .001, 0.}; in.state.last_applied_body_velocity.x = .5;
  FollowSolver solver; const auto r = solver.solve(in);
  EXPECT_FALSE(r.proposal); EXPECT_EQ(r.reason, "solver_not_solved");
  EXPECT_NE(r.solver_status, "not_run");
  const auto fresh = solver.solve(input(2)); ASSERT_TRUE(fresh.proposal) << fresh.reason; EXPECT_FALSE(fresh.used_warm);
}
TEST(Follow, ReversePathUsesActualAsymmetricVxLimit)
{
  auto in = input(); auto p = path(); std::reverse(p.poses.begin(), p.poses.end());
  in.route = PreparedCorridor::prepare(p, grid(), in.body, 4);
  FollowSolver solver; const auto r = solver.solve(in);
  ASSERT_TRUE(r.proposal) << r.reason << " / " << r.solver_status;
  EXPECT_LT(r.proposal->body_velocity.x, 0.); check_bounds(in, *r.proposal);
}
TEST(Follow, InvalidActualLimitsAndAppliedEvidenceAreRejected)
{
  for (int change = 0; change < 4; ++change) {
    auto in = input();
    if (change == 0) {in.limits.command_rate.x = 0.;}
    if (change == 1) {in.state.last_applied_body_velocity.x = .8;}
    if (change == 2) {in.state.applied_stamp_ns = 0;}
    if (change == 3) {in.progress = -1.;}
    FollowSolver solver; EXPECT_FALSE(solver.solve(in).proposal);
  }
}
