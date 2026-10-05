#include "rm_r4_nav2_controller/proposal_bridge.hpp"
#include <cmath>
#include <limits>

namespace rm_r4_nav2_controller
{
SourceContext SourceContext::capture(const BoundCycle & c, const ex::ExecutionFence & f)
{
  const auto & id = c.input.identity;
  for (const auto * s : {&f.authority_instance, &f.producer_instance, &f.producer_id, &f.execution_id,
      &f.owner_instance, &f.base_frame, &f.clock_domain, &f.clock_generation,
      &f.plan_revision, &f.body_revision, &f.limits_revision}) {
    if (s->empty() || s->size() > 128) {throw r4::ContractError("source fence identity");}
  }
  const CycleToken expected{id, c.token.plan_revision, c.token.speed_limit_revision, c.token.lifecycle_revision};
  if (!same_token(c.token, expected) || !f.active || f.authority_epoch == 0 || id.authority_epoch != f.authority_epoch ||
    id.host_instance != f.producer_instance || id.execution_id != f.execution_id || id.base_frame != f.base_frame ||
    c.input.acquired.time_since_epoch().count() <= 0 || c.input.acquired > r4::FollowClock::now())
  {throw r4::ContractError("source acquisition/fence mismatch");}
  SourceContext out; out.token_ = c.token; out.fence_ = f;
  out.provenance_ = {r4::fingerprint_follow_input(c.input), c.input.prediction.receipt_digest(),
    c.input.route.path_digest(), c.input.route.map_digest(), c.input.limits.digest(), c.input.body.digest()};
  out.epoch_ns_ = c.input.prediction.epoch_ns(); out.acquired_ = c.input.acquired; return out;
}
SourceMapping map_source_proposal(const SourceContext & c, const std::optional<CycleResult> & typed,
  const geometry_msgs::msg::TwistStamped & native, const ex::ExecutionFence & active, const SourcePublishStamp & sent)
{
  auto reject = [](const char * reason) {return SourceMapping{std::nullopt, reason};};
  if (!typed || !typed->result.proposal) {return reject("no_typed_normal_result");}
  if (!same_token(c.token_, typed->token) || !ex::same_fence(c.fence_, active) || !active.active) {
    return reject("source_token_or_fence_changed");
  }
  const auto & r = typed->result; const auto & p = *r.proposal;
  CycleToken proposal_token{p.identity, c.token_.plan_revision, c.token_.speed_limit_revision, c.token_.lifecycle_revision};
  if (!same_token(c.token_, proposal_token) || p.input_digest != c.provenance_.input_digest ||
    p.receipt_digest != c.provenance_.receipt_digest || p.path_digest != c.provenance_.path_digest ||
    p.map_digest != c.provenance_.map_digest || p.limits_digest != c.provenance_.limits_digest || p.epoch_ns != c.epoch_ns_ ||
    p.acquired != c.acquired_ || p.source_deadline != c.acquired_ + std::chrono::milliseconds(75) ||
    r.solver_status != "solved" || r.iterations <= 0 || r.iterations > 400 ||
    !std::isfinite(r.solver_seconds) || r.solver_seconds < 0 || r.solver_seconds > .015 ||
    !std::isfinite(r.elapsed_seconds) || r.elapsed_seconds < 0 || r.elapsed_seconds >= .04 ||
    !r.minimum_constraint_slack || !std::isfinite(*r.minimum_constraint_slack) || *r.minimum_constraint_slack < -1e-5)
  {return reject("result_not_from_original_bounded_cycle");}
  const auto & v = native.twist;
  const int64_t native_epoch = static_cast<int64_t>(native.header.stamp.sec) * 1000000000 + native.header.stamp.nanosec;
  if (native.header.frame_id != p.identity.base_frame || native.header.stamp.nanosec >= 1000000000 || native_epoch != p.epoch_ns ||
    v.linear.x != p.body_velocity.x || v.linear.y != p.body_velocity.y || v.angular.z != p.yaw_rate ||
    !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) || !std::isfinite(v.angular.z) ||
    v.linear.z != 0. || v.angular.x != 0. || v.angular.y != 0.)
  {return reject("native_return_differs_from_typed_result");}
  if (!sent.verified || sent.clock_domain != active.clock_domain || sent.clock_generation != active.clock_generation ||
    sent.epoch_ns < c.epoch_ns_ || sent.epoch_ns > std::numeric_limits<int64_t>::max() - 1000000000 ||
    sent.clock_drift_bound_ns < 0 || sent.clock_drift_bound_ns > 1000000 || sent.steady < c.acquired_ ||
    sent.steady.time_since_epoch().count() <= 0)
  {return reject("unverified_source_clock");}
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(sent.steady - c.acquired_).count();
  if (elapsed < static_cast<int64_t>(std::ceil(r.elapsed_seconds * 1e9)) || elapsed >= ex::source_lease_ns ||
    std::abs((sent.epoch_ns - c.epoch_ns_) - elapsed) > sent.clock_drift_bound_ns)
  {return reject("source_elapsed_clock_or_expiry");}
  return {ex::ProposalPacket{active, p.identity.cycle_sequence, ex::ProposalKind::Normal,
      {v.linear.x, v.linear.y, v.angular.z}, c.epoch_ns_, sent.epoch_ns, elapsed, ex::source_lease_ns - elapsed, c.provenance_},
    "mapped_original_acquisition_without_renewal"};
}
}  // namespace rm_r4_nav2_controller
