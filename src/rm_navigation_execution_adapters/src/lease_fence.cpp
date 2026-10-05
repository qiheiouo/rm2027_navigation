#include "rm_navigation_execution_adapters/lease_fence.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rm_navigation_execution_adapters
{
namespace
{
bool text(const std::string & s) {return !s.empty() && s.size() <= 128;}
bool stamp(int64_t n) {return n > 0 && n <= std::numeric_limits<int64_t>::max() - 1000000000;}
bool finite(Twist v) {return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.wz);}
bool digest(const std::string & s)
{return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char c) {return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');});}
bool equal(Twist a, Twist b) {return a.vx == b.vx && a.vy == b.vy && a.wz == b.wz;}
bool valid(const ExecutionFence & f)
{
  for (const auto * s : {&f.authority_instance, &f.producer_instance, &f.producer_id,
      &f.execution_id, &f.owner_instance, &f.base_frame, &f.clock_domain, &f.clock_generation,
      &f.plan_revision, &f.body_revision, &f.limits_revision}) {if (!text(*s)) {return false;}}
  return f.authority_epoch > 0;
}
}
bool same_fence(const ExecutionFence & a, const ExecutionFence & b)
{
  return a.authority_instance == b.authority_instance && a.authority_epoch == b.authority_epoch &&
    a.producer_instance == b.producer_instance && a.producer_id == b.producer_id &&
    a.execution_id == b.execution_id && a.owner_instance == b.owner_instance &&
    a.base_frame == b.base_frame && a.clock_domain == b.clock_domain &&
    a.clock_generation == b.clock_generation && a.plan_revision == b.plan_revision &&
    a.body_revision == b.body_revision && a.limits_revision == b.limits_revision && a.active == b.active;
}
ReceivedProposal::ReceivedProposal(ProposalPacket p, std::string clock, int64_t deadline, int64_t receipt)
: packet_(std::move(p)), local_clock_(std::move(clock)), deadline_(deadline), receipt_(receipt) {}
ProposalReceiptGate::ProposalReceiptGate(std::string owner, std::string clock, std::string authority)
: owner_(std::move(owner)), local_clock_(std::move(clock)), authority_(std::move(authority))
{
  if (!text(owner_) || !text(local_clock_) || !text(authority_)) {throw std::invalid_argument("registered receiver identity");}
}
bool ProposalReceiptGate::observe_fence(const ExecutionFence & f)
{
  if (!valid(f) || f.authority_instance != authority_ || f.owner_instance != owner_) {revoked_ = true; return false;}
  if (fence_ && f.authority_epoch <= fence_->authority_epoch) {
    if (!same_fence(f, *fence_)) {revoked_ = true; return false;}
    return true;  // An idempotent observation never clears an existing revoke.
  }
  fence_ = f; high_water_ = 0; last_acquired_epoch_ = 0; revoked_ = false; return true;
}
ProposalReceipt ProposalReceiptGate::receive(const ProposalPacket & p, const TransportWitness & w)
{
  auto reject = [](const char * reason) {return ProposalReceipt{std::nullopt, false, reason};};
  if (!fence_ || !fence_->active || revoked_ || !same_fence(*fence_, p.fence)) {return reject("inactive_or_mismatched_fence");}
  if (!w.verified || w.owner_instance != owner_ || w.local_clock_instance != local_clock_ ||
    w.clock_domain != fence_->clock_domain || w.clock_generation != fence_->clock_generation ||
    !stamp(w.receipt_epoch_ns) || !stamp(w.receipt_steady_ns) || w.receipt_steady_ns < last_receipt_steady_ ||
    w.clock_offset_bound_ns < 0 || w.clock_offset_bound_ns > 1000000 ||
    w.source_clock_drift_bound_ns < 0 || w.source_clock_drift_bound_ns > 1000000 ||
    w.transport_delay_bound_ns < 0 || w.transport_delay_bound_ns > 10000000)
  {revoked_ = true; return reject("unverified_transport_clock");}
  last_receipt_steady_ = w.receipt_steady_ns;
  if (p.provenance) {
    const auto & q = *p.provenance;
    for (const auto * s : {&q.input_digest, &q.receipt_digest, &q.path_digest, &q.map_digest, &q.limits_digest, &q.body_digest}) {
      if (!digest(*s)) {return reject("malformed_atomic_provenance");}
    }
  }
  if (p.cycle_sequence == 0 || p.cycle_sequence <= high_water_ || !finite(p.command) ||
    (p.kind != ProposalKind::Normal && p.kind != ProposalKind::Revoke) ||
    !stamp(p.acquired_epoch_ns) || !stamp(p.sent_epoch_ns) || p.sent_epoch_ns < p.acquired_epoch_ns ||
    p.source_elapsed_ns < 0 || p.source_elapsed_ns >= source_lease_ns ||
    p.remaining_ns != source_lease_ns - p.source_elapsed_ns ||
    p.acquired_epoch_ns < last_acquired_epoch_ ||
    (p.kind == ProposalKind::Normal && p.acquired_epoch_ns == last_acquired_epoch_))
  {return reject("invalid_or_replayed_proposal");}
  const auto ros_elapsed = p.sent_epoch_ns - p.acquired_epoch_ns;
  if (std::abs(ros_elapsed - p.source_elapsed_ns) > w.source_clock_drift_bound_ns) {
    revoked_ = true;
    return reject("source_clock_delta_mismatch");
  }
  const auto transit_upper = w.receipt_epoch_ns - p.sent_epoch_ns + w.clock_offset_bound_ns;
  // The offset witness also excludes an implausibly future sent epoch.
  if (transit_upper < 0 || transit_upper > w.transport_delay_bound_ns || transit_upper >= p.remaining_ns) {
    return reject("transit_or_source_expiry");
  }
  high_water_ = p.cycle_sequence; last_acquired_epoch_ = p.acquired_epoch_ns;
  if (p.kind == ProposalKind::Revoke) {revoked_ = true; return {std::nullopt, true, "source_revoked"};}
  const auto deadline = w.receipt_steady_ns + p.remaining_ns - transit_upper;
  return {ReceivedProposal(p, local_clock_, deadline, w.receipt_steady_ns), false, "accepted_original_source_budget"};
}
bool ProposalReceiptGate::permits(const ReceivedProposal & p) const
{
  return fence_ && fence_->active && !revoked_ && same_fence(*fence_, p.source().fence) &&
    p.local_clock_instance() == local_clock_ && p.source().cycle_sequence == high_water_;
}
SendAdmission admit_for_send(const ReceivedProposal & p, const ProposalReceiptGate & gate,
  const GeometryResult & geometry, const SendContext & c)
{
  auto reject = [](const char * reason) {return SendAdmission{std::nullopt, reason};};
  if (!gate.permits(p)) {return reject("revoked_or_superseded_proposal");}
  const auto & active = *gate.observed_fence();
  if (!valid(active) || !active.active || !same_fence(p.source().fence, active) ||
    p.source().kind != ProposalKind::Normal || c.owner_instance != active.owner_instance ||
    c.local_clock_instance != p.local_clock_instance() || !c.local_clock_verified)
  {return reject("inactive_or_mismatched_send_context");}
  if (!stamp(c.now_steady_ns) || !stamp(c.inspection_steady_ns) || !stamp(c.inspection_epoch_ns) ||
    c.inspection_steady_ns < p.receipt_steady_ns() || c.now_steady_ns < c.inspection_steady_ns ||
    c.send_budget_ns < 0 || c.send_budget_ns > 10000000 || c.output_sequence == 0 ||
    !text(c.candidate_identity) || !text(c.map_revision) || !text(c.current_frame) || !finite(c.actual_candidate) ||
    c.body_revision != active.body_revision || c.limits_revision != active.limits_revision)
  {return reject("invalid_final_candidate_context");}
  if (geometry.status != GeometryStatus::Certified || geometry.candidate_identity != c.candidate_identity ||
    !equal(geometry.candidate, c.actual_candidate) || geometry.map_revision != c.map_revision ||
    geometry.body_revision != c.body_revision || geometry.epoch_ns != c.inspection_epoch_ns ||
    geometry.frame != c.current_frame || geometry.base_frame != active.base_frame ||
    geometry.limits_revision != c.limits_revision ||
    geometry.hold_ns <= 0 || geometry.hold_ns > 50000000 ||
    geometry.processing_budget_ns <= 0 || geometry.processing_budget_ns > 10000000 ||
    !std::isfinite(geometry.elapsed_seconds) || geometry.elapsed_seconds < 0 ||
    geometry.elapsed_seconds * 1e9 > geometry.processing_budget_ns || geometry.branches.size() != 2)
  {return reject("missing_same_candidate_geometry");}
  // This local clock pair must come from the original atomic inspection seam.
  const auto spent = c.now_steady_ns - c.inspection_steady_ns;
  if (spent < static_cast<int64_t>(std::ceil(geometry.elapsed_seconds * 1e9)) ||
    spent + c.send_budget_ns > geometry.processing_budget_ns)
  {return reject("geometry_processing_or_send_budget");}
  for (const auto & branch : geometry.branches) {
    double end = 0.;
    if (branch.cover.empty()) {return reject("incomplete_geometry_cover");}
    for (const auto & interval : branch.cover) {
      if (!std::isfinite(interval.begin_seconds) || !std::isfinite(interval.end_seconds) ||
        !std::isfinite(interval.margin_lower_bound) || interval.margin_lower_bound <= 0 ||
        std::abs(interval.begin_seconds - end) > 1e-12 || interval.end_seconds <= end)
      {return reject("incomplete_geometry_cover");}
      end = interval.end_seconds;
    }
    if (std::abs(end - geometry.hold_ns * 1e-9) > 1e-12) {return reject("incomplete_geometry_cover");}
  }
  const auto deadline = std::min(p.deadline_steady_ns(), c.now_steady_ns + geometry.hold_ns);
  if (deadline <= c.now_steady_ns + c.send_budget_ns) {return reject("source_or_geometry_expired_before_send");}
  return {AdmittedCommand{p.source(), c.actual_candidate, c.owner_instance, c.candidate_identity,
      c.map_revision, c.local_clock_instance, c.output_sequence, deadline}, "admitted_without_lease_extension"};
}
}  // namespace rm_navigation_execution_adapters
