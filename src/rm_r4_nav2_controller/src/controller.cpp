#include "rm_r4_nav2_controller/controller.hpp"
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <limits>

namespace rm_r4_nav2_controller
{
namespace
{
bool same_identity(const r4::FollowIdentity & a, const r4::FollowIdentity & b)
{
  return a.host_instance == b.host_instance && a.execution_id == b.execution_id &&
    a.base_frame == b.base_frame && a.authority_epoch == b.authority_epoch && a.cycle_sequence == b.cycle_sequence;
}
bool near(double a, double b) {return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1e-12;}
bool matches(const geometry_msgs::msg::PoseStamped & p, const geometry_msgs::msg::Twist & v, const r4::FollowState & s)
{
  const auto & q = p.pose.orientation; const auto & pos = p.pose.position;
  const int64_t stamp = static_cast<int64_t>(p.header.stamp.sec) * 1000000000 + p.header.stamp.nanosec;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const double angle = 2 * std::atan2(q.z, q.w) - s.yaw;
  return p.header.stamp.nanosec < 1000000000 && p.header.frame_id == s.frame && stamp == s.pose_stamp_ns &&
    near(pos.x, s.position.x) && near(pos.y, s.position.y) && pos.z == 0. && near(q.x, 0.) && near(q.y, 0.) &&
    std::isfinite(norm) && std::abs(norm - 1.) <= 1e-9 && std::isfinite(angle) &&
    std::abs(std::atan2(std::sin(angle), std::cos(angle))) <= 1e-9 &&
    near(v.linear.x, s.measured_body_velocity.x) && near(v.linear.y, s.measured_body_velocity.y) &&
    v.linear.z == 0. && v.angular.x == 0. && v.angular.y == 0. && near(v.angular.z, s.measured_yaw_rate);
}
}
bool same_token(const CycleToken & a, const CycleToken & b)
{return same_identity(a.identity, b.identity) && a.plan_revision == b.plan_revision &&
    a.speed_limit_revision == b.speed_limit_revision && a.lifecycle_revision == b.lifecycle_revision;}
void R4Controller::invalidate()
{pending_.reset(); result_.reset(); if (solver_) {solver_->reset();}}
void R4Controller::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  cleanup();
  if (parent.expired() || name.empty() || name.size() > 128) {throw nav2_core::PlannerException("R4 configure parent/name");}
  solver_ = std::make_unique<r4::FollowSolver>(); configured_ = true;
}
void R4Controller::cleanup()
{
  invalidate(); solver_.reset(); configured_ = active_ = false; plan_digest_.clear(); speed_restricted_ = false;
  // Never reuse revision zero or a previous lifecycle token on reconfigure.
  if (plan_revision_ != std::numeric_limits<uint64_t>::max()) {++plan_revision_;}
  if (speed_revision_ != std::numeric_limits<uint64_t>::max()) {++speed_revision_;}
  if (lifecycle_revision_ != std::numeric_limits<uint64_t>::max()) {++lifecycle_revision_;}
}
void R4Controller::activate()
{
  if (!configured_ || lifecycle_revision_ == std::numeric_limits<uint64_t>::max()) {throw nav2_core::PlannerException("R4 not configured/lifecycle exhausted");}
  invalidate(); ++lifecycle_revision_; active_ = true;
}
void R4Controller::deactivate()
{invalidate(); active_ = false; if (lifecycle_revision_ != std::numeric_limits<uint64_t>::max()) {++lifecycle_revision_;}}
void R4Controller::setPlan(const nav_msgs::msg::Path & path)
{
  invalidate(); plan_digest_.clear();
  if (!configured_ || plan_revision_ == std::numeric_limits<uint64_t>::max()) {throw nav2_core::PlannerException("R4 plan lifecycle/revision");}
  ++plan_revision_;
  try {plan_digest_ = r4::PreparedCorridor::fingerprint_path(path);}
  catch (const std::exception & e) {throw nav2_core::PlannerException(e.what());}
}
void R4Controller::setSpeedLimit(const double & limit, const bool &)
{
  invalidate();
  if (speed_revision_ != std::numeric_limits<uint64_t>::max()) {++speed_revision_;}
  speed_restricted_ = !std::isfinite(limit) || limit != 0. || speed_revision_ == std::numeric_limits<uint64_t>::max();
}
bool R4Controller::bind_cycle(BoundCycle cycle)
{
  if (!configured_ || !active_ || speed_restricted_ || plan_digest_.empty() || pending_ || result_ ||
    cycle.token.plan_revision != plan_revision_ || cycle.token.speed_limit_revision != speed_revision_ ||
    cycle.token.lifecycle_revision != lifecycle_revision_ ||
    !same_identity(cycle.token.identity, cycle.input.identity) || cycle.input.route.path_digest() != plan_digest_)
  {invalidate(); return false;}
  pending_ = std::move(cycle); return true;
}
geometry_msgs::msg::TwistStamped R4Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker *)
{
  if (!configured_ || !active_ || !pending_ || speed_restricted_) {invalidate(); throw nav2_core::PlannerException("R4 missing active typed cycle");}
  auto cycle = std::move(*pending_); pending_.reset();
  if (!matches(pose, velocity, cycle.input.state)) {invalidate(); throw nav2_core::PlannerException("R4 standard pose/twist differs from typed cycle");}
  const auto acquired = cycle.input.acquired;
  auto result = solver_->solve(std::move(cycle.input));
  if (result.proposal && std::chrono::duration<double>(r4::FollowClock::now() - acquired).count() >= .04) {
    result.proposal.reset(); result.reason = "controller_total_budget"; solver_->reset();
  }
  result_ = CycleResult{cycle.token, std::move(result)};
  if (!result_->result.proposal) {throw nav2_core::PlannerException("R4 unavailable: " + result_->result.reason);}
  const auto & proposal = *result_->result.proposal;
  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = proposal.identity.base_frame;
  out.header.stamp.sec = proposal.epoch_ns / 1000000000; out.header.stamp.nanosec = proposal.epoch_ns % 1000000000;
  out.twist.linear.x = proposal.body_velocity.x; out.twist.linear.y = proposal.body_velocity.y;
  out.twist.angular.z = proposal.yaw_rate;
  result_->result.elapsed_seconds = std::chrono::duration<double>(r4::FollowClock::now() - acquired).count();
  if (result_->result.elapsed_seconds >= .04 || proposal.epoch_ns / 1000000000 > std::numeric_limits<int32_t>::max()) {
    result_->result.proposal.reset(); result_->result.reason = "controller_mapping_budget_or_stamp"; solver_->reset();
    throw nav2_core::PlannerException("R4 final mapping unavailable");
  }
  return out;
}
std::optional<CycleResult> R4Controller::take_result(const CycleToken & token)
{
  if (!result_ || !same_token(token, result_->token)) {invalidate(); return std::nullopt;}
  auto out = std::move(result_); result_.reset();
  if (out->result.proposal && r4::FollowClock::now() >= out->result.proposal->source_deadline) {
    out->result.proposal.reset(); out->result.reason = "source_expired_before_host_take"; solver_->reset();
  }
  return out;
}
}  // namespace rm_r4_nav2_controller
PLUGINLIB_EXPORT_CLASS(rm_r4_nav2_controller::R4Controller, nav2_core::Controller)
