#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "rm_navigation_execution_adapters/lease_fence.hpp"
namespace ex = rm_navigation_execution_adapters;
namespace
{
constexpr int64_t ms = 1000000, epoch = 1820000000123456789, local = 10000000000;
ex::ExecutionFence fence()
{
  return {"original-authority", "controller-host", "R4", "action-uuid-1", "original-smoother",
    "base_link", "ROS-system-clock", "clock-generation-1", "plan-1", "actual-body", "actual-limits", 1, true};
}
ex::ProposalPacket packet(ex::ExecutionFence f = fence())
{return {std::move(f), 1, ex::ProposalKind::Normal, {.2, .1, 0}, epoch, epoch + 40 * ms, 40 * ms, 35 * ms};}
ex::TransportWitness witness()
{return {"original-smoother", "receiver-steady-1", "ROS-system-clock", "clock-generation-1",
    epoch + 42 * ms, local + 42 * ms, 0, 0, 5 * ms, true};}
ex::ProposalReceiptGate gate()
{return {"original-smoother", "receiver-steady-1", "original-authority"};}
ex::GeometryResult geometry(ex::Twist actual = {.16, .08, 0})
{
  const auto t = epoch + 50 * ms;
  ex::RawCurrentGrid grid{80, 80, .05, -2, -2, std::vector<uint8_t>(6400, 0), {"odom", "grid-1", t, t, true}};
  ex::CurrentCommandInput in{ex::CurrentGridSnapshot(grid),
    {{{-.34, -.29}, {.34, -.29}, {.34, .29}, {-.34, .29}}, .02, .05, "actual-body"},
    {{-.3, -.5}, {.5, .5}, 1.2, 0, 0, 0, 0, "actual-limits"}, {0, 0, 0}, {.1, 0, 0}, actual,
    "odom", "base_link", "final-candidate-1", t, t, t, t, 50 * ms};
  return ex::inspect_current_command(in);
}
ex::SendContext send(const ex::GeometryResult & g)
{
  const auto elapsed = static_cast<int64_t>(std::ceil(g.elapsed_seconds * 1e9));
  return {"original-smoother", "receiver-steady-1", "final-candidate-1", "grid-1", "actual-body", "actual-limits", "odom",
    1, g.candidate, g.epoch_ns, local + 50 * ms, local + 50 * ms + elapsed, ms, true};
}
}
TEST(LeaseFence, ArrivalAndProducerIdentityNeverGrantExecution)
{
  auto g = gate(); EXPECT_FALSE(g.receive(packet(), witness()).proposal);
  ASSERT_TRUE(g.observe_fence(fence())); auto p = packet(); p.fence.producer_id = "Spin";
  EXPECT_FALSE(g.receive(p, witness()).proposal);
  auto unknown = fence(); unknown.authority_instance = "unregistered"; EXPECT_FALSE(g.observe_fence(unknown));
  EXPECT_FALSE(g.receive(packet(), witness()).proposal);
}
TEST(LeaseFence, AcquisitionDeadlineConsumesComputeTransitAndClockOffset)
{
  auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto w = witness(); w.clock_offset_bound_ns = ms;
  auto r = g.receive(packet(), w); ASSERT_TRUE(r.proposal) << r.reason;
  EXPECT_EQ(r.proposal->deadline_steady_ns(), local + 74 * ms);
  EXPECT_EQ(r.proposal->source().acquired_epoch_ns, epoch);
  EXPECT_NE(r.proposal->deadline_steady_ns(), w.receipt_steady_ns + ex::source_lease_ns);
}
TEST(LeaseFence, DuplicateReorderAndNewSequenceWithOldAcquisitionNeverRenew)
{
  auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto p = packet(); auto r = g.receive(p, witness()); ASSERT_TRUE(r.proposal);
  auto w = witness(); w.receipt_epoch_ns += ms; w.receipt_steady_ns += ms;
  EXPECT_FALSE(g.receive(p, w).proposal); p.cycle_sequence = 2; EXPECT_FALSE(g.receive(p, w).proposal);
  p.cycle_sequence = 0; EXPECT_FALSE(g.receive(p, w).proposal);
  EXPECT_TRUE(g.permits(*r.proposal)); EXPECT_EQ(r.proposal->deadline_steady_ns(), local + 75 * ms);
}
TEST(LeaseFence, OldExecutionNormalZeroAndRevokeCannotCoverNewFence)
{
  for (int kind = 0; kind < 3; ++kind) {
    auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto old = packet();
    auto next = fence(); next.authority_epoch = 2; next.producer_id = "Spin"; next.execution_id = "spin-action";
    ASSERT_TRUE(g.observe_fence(next)); if (kind == 1) {old.command = {};}
    if (kind == 2) {old.kind = ex::ProposalKind::Revoke;}
    EXPECT_FALSE(g.receive(old, witness()).proposal); EXPECT_FALSE(g.receive(old, witness()).revoked);
    auto p = packet(next); auto r = g.receive(p, witness()); ASSERT_TRUE(r.proposal); EXPECT_TRUE(g.permits(*r.proposal));
  }
}
TEST(LeaseFence, RevocationInvalidatesHeldProposalAndCannotReopenSameFence)
{
  auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto p = packet(); auto r = g.receive(p, witness()); ASSERT_TRUE(r.proposal);
  p.kind = ex::ProposalKind::Revoke; p.cycle_sequence = 2;
  EXPECT_TRUE(g.receive(p, witness()).revoked); EXPECT_FALSE(g.permits(*r.proposal));
  ASSERT_TRUE(g.observe_fence(fence())); p.kind = ex::ProposalKind::Normal; p.cycle_sequence = 3; p.acquired_epoch_ns += ms;
  EXPECT_FALSE(g.receive(p, witness()).proposal);
  auto next = fence(); next.authority_epoch = 2; ASSERT_TRUE(g.observe_fence(next));
  EXPECT_TRUE(g.receive(packet(next), witness()).proposal);
}
TEST(LeaseFence, OwnerRestartNeedsExplicitNewTargetRegistration)
{
  ex::ProposalReceiptGate next("owner-restarted", "new-local-clock", "original-authority");
  EXPECT_FALSE(next.observe_fence(fence())); EXPECT_FALSE(next.receive(packet(), witness()).proposal);
  auto f = fence(); f.owner_instance = "owner-restarted"; f.authority_epoch = 2;
  ASSERT_TRUE(next.observe_fence(f)); auto w = witness(); w.owner_instance = f.owner_instance; w.local_clock_instance = "new-local-clock";
  EXPECT_FALSE(next.receive(packet(), w).proposal); EXPECT_TRUE(next.receive(packet(f), w).proposal);
}
TEST(LeaseFence, SameEpochMutationOrRollbackFailsClosed)
{
  for (int change = 0; change < 3; ++change) {
    auto g = gate(); auto f = fence(); f.authority_epoch = 2; ASSERT_TRUE(g.observe_fence(f));
    auto r = g.receive(packet(f), witness()); ASSERT_TRUE(r.proposal);
    auto bad = f; if (change == 0) {bad.authority_epoch = 1;} if (change == 1) {bad.limits_revision = "new";}
    if (change == 2) {bad.active = false;}
    EXPECT_FALSE(g.observe_fence(bad)); EXPECT_FALSE(g.permits(*r.proposal));
    EXPECT_TRUE(g.observe_fence(f)); EXPECT_FALSE(g.permits(*r.proposal));
  }
}
TEST(LeaseFence, UnknownResetSkewAndClockRollbackRejectAndRevoke)
{
  for (int change = 0; change < 7; ++change) {
    auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto r = g.receive(packet(), witness()); ASSERT_TRUE(r.proposal);
    auto w = witness(); auto p = packet(); p.cycle_sequence = 2; p.acquired_epoch_ns += ms;
    if (change == 0) {w.verified = false;} if (change == 1) {w.clock_generation = "reset";}
    if (change == 2) {w.local_clock_instance = "another-process";} if (change == 3) {w.clock_offset_bound_ns = ms + 1;}
    if (change == 4) {w.receipt_steady_ns -= 1;} if (change == 5) {w.receipt_epoch_ns = -1;}
    if (change == 6) {p.sent_epoch_ns += 2 * ms;}
    EXPECT_FALSE(g.receive(p, w).proposal); EXPECT_FALSE(g.permits(*r.proposal));
  }
}
TEST(LeaseFence, LatePacketOrForgedRemainingCannotRebaseLease)
{
  for (int change = 0; change < 6; ++change) {
    auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto p = packet(); auto w = witness();
    if (change == 0) {p.remaining_ns = 75 * ms;} if (change == 1) {w.receipt_epoch_ns = epoch + 76 * ms;}
    if (change == 2) {w.transport_delay_bound_ns = ms;} if (change == 3) {p.sent_epoch_ns += 4 * ms;}
    if (change == 4) {p.source_elapsed_ns = 75 * ms; p.remaining_ns = 0;}
    if (change == 5) {p.command.vx = std::numeric_limits<double>::quiet_NaN();}
    EXPECT_FALSE(g.receive(p, w).proposal);
  }
}
TEST(LeaseFence, ActualSmoothingResultUsesSameGeometryAndTruncatedOriginalLease)
{
  auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto r = g.receive(packet(), witness()); ASSERT_TRUE(r.proposal);
  auto proof = geometry(); ASSERT_EQ(proof.status, ex::GeometryStatus::Certified) << proof.reason;
  auto c = send(proof); auto a = ex::admit_for_send(*r.proposal, g, proof, c); ASSERT_TRUE(a.command) << a.reason;
  EXPECT_DOUBLE_EQ(a.command->actual_command.vx, .16); EXPECT_DOUBLE_EQ(a.command->source.command.vx, .2);
  EXPECT_EQ(a.command->valid_until_steady_ns, local + 75 * ms);
  EXPECT_LT(a.command->valid_until_steady_ns - c.now_steady_ns, 25 * ms + 1);
  EXPECT_EQ(a.command->source.acquired_epoch_ns, epoch);
}
TEST(LeaseFence, MismatchedCandidateMapBodyLimitsOrIncompleteCoverIsUnavailable)
{
  auto proof = geometry(); ASSERT_EQ(proof.status, ex::GeometryStatus::Certified) << proof.reason;
  for (int change = 0; change < 13; ++change) {
    auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto r = g.receive(packet(), witness()); ASSERT_TRUE(r.proposal);
    auto c = send(proof); auto bad = proof;
    if (change == 0) {c.actual_candidate.vx += .001;} if (change == 1) {c.candidate_identity = "old-tick";}
    if (change == 2) {c.map_revision = "new-map";} if (change == 3) {c.body_revision = "new-body";}
    if (change == 4) {c.limits_revision = "new-limits";} if (change == 5) {c.inspection_epoch_ns += 1;}
    if (change == 6) {bad.branches.pop_back();} if (change == 7) {bad.branches[0].cover[0].begin_seconds += .001;}
    if (change == 8) {bad.status = ex::GeometryStatus::Collision;} if (change == 9) {bad.branches[0].cover[0].margin_lower_bound = 0;}
    if (change == 10) {bad.base_frame = "wrong-base";} if (change == 11) {bad.frame = "map";}
    if (change == 12) {bad.limits_revision = "old-limits";}
    EXPECT_FALSE(ex::admit_for_send(*r.proposal, g, bad, c).command);
  }
}
TEST(LeaseFence, ExpiryProcessingAndClockFailuresNeverProduceAppliedCommand)
{
  auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto r = g.receive(packet(), witness()); ASSERT_TRUE(r.proposal);
  auto proof = geometry(); ASSERT_EQ(proof.status, ex::GeometryStatus::Certified) << proof.reason;
  for (int change = 0; change < 6; ++change) {
    auto c = send(proof); auto bad = proof;
    if (change == 0) {c.now_steady_ns = local + 75 * ms;} if (change == 1) {c.now_steady_ns = c.inspection_steady_ns + 10 * ms;}
    if (change == 2) {c.local_clock_verified = false;} if (change == 3) {c.local_clock_instance = "cross-process";}
    if (change == 4) {c.send_budget_ns = 10 * ms + 1;} if (change == 5) {bad.processing_budget_ns = 0;}
    EXPECT_FALSE(ex::admit_for_send(*r.proposal, g, bad, c).command);
  }
  auto revoked = packet(); revoked.kind = ex::ProposalKind::Revoke; revoked.cycle_sequence = 2;
  ASSERT_TRUE(g.receive(revoked, witness()).revoked);
  EXPECT_FALSE(ex::admit_for_send(*r.proposal, g, proof, send(proof)).command);
}
TEST(LeaseFence, FortyMsComputeAndFiftyMsOwnerRetainTwentyFiveMsGap)
{
  auto g = gate(); ASSERT_TRUE(g.observe_fence(fence())); auto first = g.receive(packet(), witness()); ASSERT_TRUE(first.proposal);
  // Deterministic schedule counterexample, not timing measurement or a new owner.
  EXPECT_EQ(first.proposal->deadline_steady_ns(), local + 75 * ms);
  auto next = packet(); next.cycle_sequence = 2; next.acquired_epoch_ns += 50 * ms; next.sent_epoch_ns += 50 * ms;
  auto w = witness(); w.receipt_epoch_ns += 50 * ms; w.receipt_steady_ns += 50 * ms;
  auto second = g.receive(next, w); ASSERT_TRUE(second.proposal);
  EXPECT_FALSE(g.permits(*first.proposal)); EXPECT_TRUE(g.permits(*second.proposal));
  EXPECT_EQ(second.proposal->deadline_steady_ns(), local + 125 * ms);
  EXPECT_EQ((local + 100 * ms) - first.proposal->deadline_steady_ns(), 25 * ms);
}
