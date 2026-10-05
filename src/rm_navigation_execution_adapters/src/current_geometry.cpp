#include "rm_navigation_execution_adapters/current_geometry.hpp"
#include "rm_tdt_planner/pose_geometry.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rm_navigation_execution_adapters
{
namespace pg = rm_tdt_planner::pose_geometry;
namespace
{
using Clock = std::chrono::steady_clock;
bool finite(Point p) {return std::isfinite(p.x) && std::isfinite(p.y);}
bool finite(Twist v) {return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.wz);}
bool text(const std::string & s) {return !s.empty() && s.size() <= 128;}
double elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now() - t).count();}
bool bounded(Twist v, const MotionBounds & b)
{
  return finite(v) && v.vx >= b.lower.x && v.vx <= b.upper.x &&
    v.vy >= b.lower.y && v.vy <= b.upper.y && std::abs(v.wz) <= b.yaw_rate;
}
bool fresh_through(int64_t stamp, int64_t epoch, int64_t ttl, int64_t future)
{return stamp > 0 && stamp <= epoch && epoch - stamp <= ttl - future;}
}
struct CurrentGridSnapshot::Impl
{
  const GridMetadata metadata;
  const pg::RawSnapshot raw;
  explicit Impl(RawCurrentGrid input)
  : metadata(std::move(input.metadata)), raw(pg::RawCostmapInput{input.width, input.height,
      input.resolution, input.origin_x, input.origin_y, std::move(input.costs)})
  {
    if (!text(metadata.frame) || !text(metadata.revision) || metadata.update_ns <= 0 ||
      metadata.receipt_ns < metadata.update_ns) {throw std::invalid_argument("current grid metadata");}
  }
};
CurrentGridSnapshot::CurrentGridSnapshot(RawCurrentGrid input) : impl_(std::make_shared<const Impl>(std::move(input))) {}
const GridMetadata & CurrentGridSnapshot::metadata() const {return impl_->metadata;}
struct GeometryAccess
{
  static const pg::RawSnapshot & raw(const CurrentGridSnapshot & g) {return g.impl_->raw;}
};

Pose integrate_held_body_twist(Pose p, Twist v, double duration)
{
  if (!finite(Point{p.x, p.y}) || !std::isfinite(p.yaw) || !finite(v) ||
    !std::isfinite(duration) || duration < 0 || duration > .05)
  {throw std::invalid_argument("finite held body twist interval <=50ms");}
  const double angle = v.wz * duration;
  const double sinc = angle == 0 ? 1. : std::sin(angle) / angle;
  const double cosc = angle == 0 ? 0. : 2 * std::sin(angle / 2) * std::sin(angle / 2) / angle;
  const double dx = duration * (sinc * v.vx - cosc * v.vy);
  const double dy = duration * (cosc * v.vx + sinc * v.vy);
  return {p.x + std::cos(p.yaw) * dx - std::sin(p.yaw) * dy,
    p.y + std::sin(p.yaw) * dx + std::cos(p.yaw) * dy, p.yaw + angle};
}

GeometryResult inspect_current_command(const CurrentCommandInput & in, const GeometryPolicy & cfg)
{
  const auto started = Clock::now(); GeometryResult result;
  result.candidate_identity = in.candidate_identity; result.candidate = in.candidate;
  result.map_revision = in.grid.metadata().revision; result.body_revision = in.body.revision;
  result.frame = in.frame; result.base_frame = in.base_frame; result.limits_revision = in.bounds.revision;
  result.epoch_ns = in.epoch_ns; result.hold_ns = in.hold_ns;
  if (std::isfinite(cfg.budget_seconds) && cfg.budget_seconds > 0 && cfg.budget_seconds <= .010) {
    result.processing_budget_ns = static_cast<int64_t>(std::ceil(cfg.budget_seconds * 1e9));
  }
  auto finish = [&](GeometryStatus status, const std::string & reason) {
      result.status = status; result.reason = reason; result.elapsed_seconds = elapsed(started);
      if (std::isfinite(cfg.budget_seconds) && cfg.budget_seconds > 0 &&
        result.elapsed_seconds > cfg.budget_seconds)
      {result.status = GeometryStatus::Unavailable; result.reason = "geometry_time_budget";}
      if (result.status != GeometryStatus::Certified) {result.branches.clear();}
      return result;
    };
  try {
    const auto & grid = in.grid.metadata(); const auto & b = in.bounds;
    if (!std::isfinite(cfg.budget_seconds) || cfg.budget_seconds <= 0 || cfg.budget_seconds > .010 ||
      cfg.max_intervals < 1 || cfg.max_intervals > 8191 || cfg.max_cell_checks < 1 || cfg.max_cell_checks > 2000000 ||
      cfg.max_depth > 20 || cfg.state_ttl_ns <= 0 || cfg.state_ttl_ns > 100000000 ||
      cfg.update_ttl_ns <= 0 || cfg.update_ttl_ns > 150000000 || cfg.receipt_ttl_ns <= 0 || cfg.receipt_ttl_ns > 250000000 ||
      in.epoch_ns <= 0 || in.epoch_ns > std::numeric_limits<int64_t>::max() - 1000000000 ||
      in.hold_ns <= 0 || in.hold_ns > 50000000 || !text(in.frame) || !text(in.base_frame) ||
      !text(in.candidate_identity) || !text(in.body.revision) || !text(b.revision) || in.frame != grid.frame || !grid.current ||
      in.tf_stamp_ns != in.pose_stamp_ns || !finite(Point{in.pose.x, in.pose.y}) || !std::isfinite(in.pose.yaw) ||
      !finite(b.lower) || !finite(b.upper) || b.lower.x > 0 || b.lower.y > 0 || b.upper.x < 0 || b.upper.y < 0 ||
      b.lower.x >= b.upper.x || b.lower.y >= b.upper.y || b.lower.x < -3 || b.lower.y < -3 || b.upper.x > 3 || b.upper.y > 3 ||
      !std::isfinite(b.yaw_rate) || b.yaw_rate <= 0 || b.yaw_rate > 3 ||
      !bounded(in.measured, b) || !bounded(in.candidate, b) ||
      !std::isfinite(in.body.padding) || in.body.padding <= 0 || in.body.padding > .5 ||
      !std::isfinite(in.body.clearance) || in.body.clearance < .05 || in.body.clearance > .5 ||
      in.body.padded_footprint.size() < 3 || in.body.padded_footprint.size() > 32)
    {return finish(GeometryStatus::Unavailable, "invalid_current_geometry_contract");}
    for (double error : {b.position_error, b.yaw_error, b.tracking_position_error, b.tracking_yaw_error}) {
      if (!std::isfinite(error) || error < 0 || error > 1.) {return finish(GeometryStatus::Unavailable, "invalid_error_tube");}
    }
    const int64_t future = in.hold_ns + static_cast<int64_t>(std::ceil(cfg.budget_seconds * 1e9));
    if (!fresh_through(in.pose_stamp_ns, in.epoch_ns, cfg.state_ttl_ns, future) ||
      !fresh_through(in.velocity_stamp_ns, in.epoch_ns, cfg.state_ttl_ns, future) ||
      !fresh_through(grid.update_ns, in.epoch_ns, cfg.update_ttl_ns, future) ||
      !fresh_through(grid.receipt_ns, in.epoch_ns, cfg.receipt_ttl_ns, future))
    {return finish(GeometryStatus::Unavailable, "input_expiry_before_interval_end");}
    std::vector<rm_tdt_planner::Point> footprint; double radius = 0.;
    for (auto p : in.body.padded_footprint) {
      if (!finite(p) || std::hypot(p.x, p.y) > 2.) {return finish(GeometryStatus::Unavailable, "invalid_actual_footprint");}
      radius = std::max(radius, std::hypot(p.x, p.y)); footprint.push_back({p.x, p.y});
    }
    const double hold = in.hold_ns * 1e-9;
    const double age = (in.epoch_ns - in.pose_stamp_ns) * 1e-9 + cfg.budget_seconds;
    const double speed_bound = std::hypot(std::max(-b.lower.x, b.upper.x), std::max(-b.lower.y, b.upper.y));
    const double position_reserve = b.position_error + speed_bound * age + b.tracking_position_error;
    const double yaw_reserve = radius * (b.yaw_error + b.yaw_rate * age + b.tracking_yaw_error);
    for (int branch = 0; branch < 2; ++branch) {
      const auto velocity = branch == 0 ? in.measured : in.candidate;
      const auto end = integrate_held_body_twist(in.pose, velocity, hold);
      const double chord_error = std::hypot(velocity.vx, velocity.vy) * std::abs(velocity.wz) * hold * hold / 8;
      pg::Limits limits;
      limits.clearance = in.body.clearance + position_reserve + yaw_reserve + chord_error;
      limits.centre_reserve = position_reserve + chord_error;
      limits.time_budget_seconds = cfg.budget_seconds - elapsed(started);
      limits.max_intervals = cfg.max_intervals - std::min(cfg.max_intervals, result.intervals_examined);
      limits.max_cell_checks = cfg.max_cell_checks - std::min(cfg.max_cell_checks, result.cell_checks);
      limits.max_depth = cfg.max_depth;
      if (limits.time_budget_seconds <= 0 || limits.max_intervals == 0 || limits.max_cell_checks == 0) {
        return finish(GeometryStatus::Unavailable, "geometry_work_budget");
      }
      const pg::Motion motion{{in.pose.x, in.pose.y, in.pose.yaw}, {end.x, end.y, end.yaw}, velocity.wz * hold};
      const auto evidence = pg::certify(GeometryAccess::raw(in.grid), footprint, motion, limits);
      result.cell_checks += evidence.cell_checks; result.intervals_examined += evidence.intervals_examined;
      if (evidence.status != pg::Status::Certified) {
        result.collision_branch = evidence.status == pg::Status::Collision ? branch : -1;
        return finish(evidence.status == pg::Status::Collision ? GeometryStatus::Collision : GeometryStatus::Unavailable,
          evidence.reason);
      }
      BranchEvidence part{chord_error, limits.centre_reserve, limits.clearance - in.body.clearance, {}};
      for (const auto & interval : evidence.intervals) {
        part.cover.push_back({interval.begin * hold, interval.end * hold, interval.constraint_margin_lower_bound});
      }
      result.branches.push_back(std::move(part));
    }
    return finish(GeometryStatus::Certified, "current_geometry_under_declared_model");
  } catch (const std::exception & error) {return finish(GeometryStatus::Unavailable, error.what());}
}
}  // namespace rm_navigation_execution_adapters
