#pragma once
#include "rm_competition_interfaces/msg/dynamic_obstacle_prediction_array.hpp"
#include "rm_dynamic_obstacle_critic/geometry.hpp"
#include "tf2_ros/buffer.h"
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace rm_dynamic_obstacle_critic {
using Array = rm_competition_interfaces::msg::DynamicObstaclePredictionArray;
using Obstacle = rm_competition_interfaces::msg::DynamicObstaclePrediction;
inline double seconds(const builtin_interfaces::msg::Time &t) {
  return t.sec + t.nanosec * 1e-9;
}
struct Limits {
  double max_age{0.4}, max_observation_age{0.4}, max_speed{3.0},
      max_extent{3.0};
  double minimum_radius{0.36}, jump_tolerance{0.5}, max_tf_age{0.1};
  std::string input_frame{"map"};
  size_t max_tracks{64};
};
inline std::string validate(const Array &msg, double now, const Limits &lim) {
  if (msg.schema != Array::SCHEMA ||
      msg.authority != Array::AUTHORITY_SHADOW_ONLY ||
      msg.header.frame_id != lim.input_frame)
    return "schema_or_frame";
  if (!msg.complete || msg.total_track_count != msg.tracks.size() ||
      msg.tracks.size() > lim.max_tracks)
    return "incomplete_or_overflow";
  const double stamp = seconds(msg.header.stamp), age = now - stamp;
  if (!std::isfinite(now) || stamp <= 0 || age < 0 || age > lim.max_age)
    return "stale_or_future_source";
  if (!std::isfinite(msg.prediction_dt) || msg.prediction_dt <= 0 ||
      msg.prediction_steps == 0)
    return "prediction_grid";
  std::map<uint64_t, bool> ids;
  for (const auto &t : msg.tracks) {
    if (ids.count(t.track_id))
      return "duplicate_id";
    ids[t.track_id] = true;
    const double values[] = {t.position.x, t.position.y, t.velocity.x,
                             t.velocity.y, t.size.x,     t.size.y};
    for (double v : values)
      if (!std::isfinite(v))
        return "nonfinite_track";
    if (t.state != Obstacle::STATE_TENTATIVE &&
        t.state != Obstacle::STATE_CONFIRMED &&
        t.state != Obstacle::STATE_COASTING)
      return "invalid_state";
    if (t.size.x <= 0 || t.size.y <= 0 || t.size.x > lim.max_extent ||
        t.size.y > lim.max_extent ||
        std::hypot(t.velocity.x, t.velocity.y) > lim.max_speed)
      return "size_or_speed";
    const double observed = seconds(t.last_observation_stamp);
    if (observed <= 0 || observed > stamp ||
        now - observed > lim.max_observation_age)
      return "stale_observation";
  }
  return "";
}
class InputCache {
public:
  void receive(Array::ConstSharedPtr next, const Limits &lim) {
    std::lock_guard<std::mutex> lock(mutex_);
    reason_.clear();
    if (next->tracks.size() > lim.max_tracks) {
      reason_ = "overflow";
      latest_ = std::move(next);
      return;
    }
    if (latest_ && latest_->tracks.size() <= lim.max_tracks &&
        latest_->header.frame_id == next->header.frame_id) {
      double dt = seconds(next->header.stamp) - seconds(latest_->header.stamp);
      if (dt <= 0)
        reason_ = "nonmonotonic_source";
      else
        for (const auto &a : latest_->tracks)
          for (const auto &b : next->tracks) {
            if (a.track_id == b.track_id &&
                std::hypot(b.position.x - a.position.x,
                           b.position.y - a.position.y) >
                    lim.max_speed * dt + lim.jump_tolerance)
              reason_ = "track_jump";
          }
    }
    // Even invalid input replaces the old snapshot: never silently keep stale
    // velocity.
    latest_ = std::move(next);
  }
  Array::ConstSharedPtr get(std::string &reason) const {
    std::lock_guard<std::mutex> lock(mutex_);
    reason = reason_;
    return latest_;
  }

private:
  mutable std::mutex mutex_;
  Array::ConstSharedPtr latest_;
  std::string reason_;
};
struct Rigid2D {
  double x{0}, y{0}, yaw{0};
  Point apply(Point p) const {
    return {x + std::cos(yaw) * p.x - std::sin(yaw) * p.y,
            y + std::sin(yaw) * p.x + std::cos(yaw) * p.y};
  }
};
inline Rigid2D frame_transform(tf2_ros::Buffer &tf, const std::string &target,
                               const std::string &source,
                               const rclcpp::Time &now, double max_age = 0.1) {
  if (target == source)
    return {};
  // World-frame alignment uses the most recent correction with an explicit age
  // bound. Static TF has stamp zero. Moving sensor/body source frames are
  // outside this API.
  auto tr = tf.lookupTransform(target, source,
                               rclcpp::Time(0, 0, now.get_clock_type()),
                               rclcpp::Duration::from_seconds(0.0));
  const double stamp = seconds(tr.header.stamp);
  if (stamp > 0 && (now.seconds() < stamp || now.seconds() - stamp > max_age))
    throw std::invalid_argument("stale/future world-frame transform");
  const auto &q = tr.transform.rotation;
  const auto &p = tr.transform.translation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-3 ||
      std::abs(q.x) > 1e-3 || std::abs(q.y) > 1e-3 || !std::isfinite(p.x) ||
      !std::isfinite(p.y))
    throw std::invalid_argument("nonplanar/invalid TF");
  return {p.x, p.y, std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)};
}
inline Point predict(const Obstacle &t, double source_age, double future,
                     const Rigid2D &tr = {}) {
  return tr.apply({t.position.x + t.velocity.x * (source_age + future),
                   t.position.y + t.velocity.y * (source_age + future)});
}
inline double obstacle_radius(const Obstacle &t, const Limits &lim) {
  return std::max(lim.minimum_radius, 0.5 * std::hypot(t.size.x, t.size.y));
}
struct CostParameters {
  double weight{1.0}, influence_distance{0.4}, safety_margin{0.02},
      collision_cost{10000.0};
};
inline double penalty(double clearance, const CostParameters &p) {
  // Both squared hinges are continuous, including at safety-margin and contact.
  const double near =
      std::max(0.0, (p.influence_distance - clearance) / p.influence_distance);
  const double danger =
      std::max(0.0, (p.safety_margin - clearance) / p.safety_margin);
  return p.weight * (near * near + p.collision_cost * danger * danger);
}
struct Risk {
  double cost{0}, minimum_clearance{std::numeric_limits<double>::infinity()};
  double collision_time{std::numeric_limits<double>::infinity()};
};
inline double clearance(const std::vector<Point> &placed, const Array &msg,
                        const Limits &lim, double age, double future,
                        const Rigid2D &tr = {}) {
  double result = std::numeric_limits<double>::infinity();
  for (const auto &t : msg.tracks) {
    if (t.state == Obstacle::STATE_TENTATIVE)
      continue; // Current costmap still covers these.
    result =
        std::min(result, circle_clearance(placed, predict(t, age, future, tr),
                                          obstacle_radius(t, lim)));
  }
  return result;
}
inline Risk score(const std::vector<Pose> &rollout,
                  const std::vector<Point> &footprint, const Array &msg,
                  const Limits &lim, const CostParameters &cost, double age,
                  double dt, const Rigid2D &tr = {}) {
  Risk r;
  for (size_t k = 0; k < rollout.size(); ++k) {
    const auto &p = rollout[k];
    const double time = (k + 1) * dt;
    double d = clearance(transform(footprint, p.x, p.y, p.yaw), msg, lim, age,
                         time, tr);
    r.minimum_clearance = std::min(r.minimum_clearance, d);
    if (d <= cost.safety_margin)
      r.collision_time = std::min(r.collision_time, time);
    r.cost += penalty(d, cost) * dt;
  }
  return r;
}
} // namespace rm_dynamic_obstacle_critic
