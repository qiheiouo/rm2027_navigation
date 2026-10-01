#pragma once
#include "rm_dynamic_obstacle_critic/model.hpp"
#include <functional>
namespace rm_dynamic_obstacle_critic {
struct GuardParameters {
  double horizon{0.5}, dt{0.02}, response_delay{0.1}, linear_deceleration{1.0},
      angular_deceleration{2.0};
  double safety_margin{0.02};
  size_t max_steps{512};
};
struct GuardResult {
  bool pass{true};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  double collision_time{std::numeric_limits<double>::infinity()};
  std::string reason{"clear"};
  int collision_branch{-1}; // 0: measured response; 1: proposed command.
  Pose collision_pose{};
  Velocity collision_velocity{};
  double static_reserve{0}, dynamic_reserve{0};
};
enum class GuardPaths { All, MeasuredOnly, ProposedOnly };
inline double approach_zero(double v, double deceleration, double dt) {
  return std::copysign(std::max(0.0, std::abs(v) - deceleration * dt), v);
}
inline GuardResult check_command(
    Pose start, Velocity measured, Velocity command,
    const std::vector<Point> &footprint, const Array &obstacles,
    const Limits &lim, double age, const Rigid2D &tr,
    const GuardParameters &cfg,
    const std::function<bool(const std::vector<Point> &, double)> &static_clear,
    GuardPaths paths = GuardPaths::All) {
  GuardResult r;
  double radius = 0, obstacle_speed = 0;
  for (auto p : footprint)
    radius = std::max(radius, std::hypot(p.x, p.y));
  for (auto t : obstacles.tracks)
    obstacle_speed =
        std::max(obstacle_speed, std::hypot(t.velocity.x, t.velocity.y));
  double stop = std::max({std::hypot(command.x, command.y),
                          std::hypot(measured.x, measured.y)}) /
                cfg.linear_deceleration;
  stop =
      std::max(stop, std::max(std::abs(command.yaw), std::abs(measured.yaw)) /
                         cfg.angular_deceleration);
  const double horizon = std::max(cfg.horizon, cfg.response_delay) + stop;
  if (!std::isfinite(horizon / cfg.dt) || horizon / cfg.dt > cfg.max_steps) {
    r.pass = false;
    r.reason = "simulation_budget_exceeded";
    return r;
  }
  // Two independent bounded-response paths: existing measured motion and
  // commanded motion. Their union is checked, not averaged. Actuator tracking
  // between them still requires validation.
  for (size_t branch = 0; branch < 2; ++branch) {
    if ((paths == GuardPaths::MeasuredOnly && branch != 0) ||
        (paths == GuardPaths::ProposedOnly && branch != 1))
      continue;
    const auto initial = branch == 0 ? measured : command;
    Pose p = start;
    Velocity v = initial;
    const double swept =
        (std::hypot(v.x, v.y) + radius * std::abs(v.yaw)) * cfg.dt;
    const double reserve =
        swept + obstacle_speed * cfg.dt; // full interval Lipschitz reserve
    for (size_t k = 0; k <= static_cast<size_t>(std::ceil(horizon / cfg.dt));
         ++k) {
      double time = k * cfg.dt;
      auto poly = transform(footprint, p.x, p.y, p.yaw);
      const double d = clearance(poly, obstacles, lim, age, time, tr) - reserve;
      r.minimum_clearance = std::min(r.minimum_clearance, d);
      // Preserve the original short-circuit order. Diagnostic capture must not
      // change which check rejects a command or the pass/brake decision.
      if (d <= cfg.safety_margin || !static_clear(poly, swept)) {
        r.pass = false;
        r.collision_time = std::min(r.collision_time, time);
        r.reason = d <= cfg.safety_margin ? "dynamic_collision"
                                          : "static_collision_or_unknown";
        r.collision_branch = static_cast<int>(branch);
        r.collision_pose = p;
        r.collision_velocity = v;
        r.static_reserve = swept;
        r.dynamic_reserve = reserve;
        return r;
      }
      p = advance(p, v, cfg.dt);
      // Proposed command is held for the short horizon, then brake. Measured
      // path brakes after response_delay. Check the entire stopping tail; no
      // horizon truncation.
      const bool measured_path = branch == 0;
      if (time + cfg.dt >= (measured_path ? cfg.response_delay : cfg.horizon)) {
        double speed = std::hypot(v.x, v.y),
               next = std::max(0.0, speed - cfg.linear_deceleration * cfg.dt);
        if (speed > 0) {
          v.x *= next / speed;
          v.y *= next / speed;
        }
        v.yaw = approach_zero(v.yaw, cfg.angular_deceleration, cfg.dt);
      }
    }
  }
  return r;
}
} // namespace rm_dynamic_obstacle_critic
