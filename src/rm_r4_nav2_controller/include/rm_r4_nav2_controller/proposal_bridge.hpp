#pragma once
#include "rm_r4_nav2_controller/controller.hpp"
#include <rm_navigation_execution_adapters/lease_fence.hpp>

namespace rm_r4_nav2_controller
{
namespace ex = rm_navigation_execution_adapters;
struct SourcePublishStamp
{
  r4::FollowClock::time_point steady;
  int64_t epoch_ns{}, clock_drift_bound_ns{};
  std::string clock_domain, clock_generation;
  bool verified{};
};
struct SourceMapping {std::optional<ex::ProposalPacket> packet; std::string reason;};
class SourceContext
{
public:
  // Called at the original host acquire/bind seam. Consumes, never issues, grant.
  static SourceContext capture(const BoundCycle & cycle, const ex::ExecutionFence & fence);
private:
  CycleToken token_;
  ex::ExecutionFence fence_;
  ex::ProposalProvenance provenance_;
  int64_t epoch_ns_{};
  r4::FollowClock::time_point acquired_;
  SourceContext() = default;
  friend SourceMapping map_source_proposal(const SourceContext &, const std::optional<CycleResult> &,
    const geometry_msgs::msg::TwistStamped &, const ex::ExecutionFence &, const SourcePublishStamp &);
};
SourceMapping map_source_proposal(const SourceContext & context, const std::optional<CycleResult> & result,
  const geometry_msgs::msg::TwistStamped & native_return, const ex::ExecutionFence & current_fence,
  const SourcePublishStamp & publication);
}  // namespace rm_r4_nav2_controller
