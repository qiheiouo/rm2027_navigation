#include "rm_r4_prediction_consumption/consumption.hpp"
#include "digest.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace rm_r4_prediction_consumption
{
namespace
{
bool finite(Vec2 p) {return std::isfinite(p.x) && std::isfinite(p.y);}
double norm(Vec2 p) {return std::hypot(p.x, p.y);}
Vec2 subtract(Vec2 a, Vec2 b) {return {a.x - b.x, a.y - b.y};}
double dot(Vec2 a, Vec2 b) {return a.x * b.x + a.y * b.y;}
int64_t epoch(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {throw ContractError("ROS epoch");}
  return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}
template<class Point> Vec2 planar(const Point & p)
{
  if (!std::isfinite(p.z) || p.z != 0.0 || !finite({p.x, p.y})) {
    throw ContractError("finite planar value");
  }
  return {p.x, p.y};
}
}

Bounds BodyPolicy::support() const
{
  if (footprint.size() < 3 || footprint.size() > 64 || !std::isfinite(padding) ||
    padding <= 0 || padding > 0.5 || !std::isfinite(yaw) ||
    !std::isfinite(static_clearance) || static_clearance < 0.05 || static_clearance > 0.5)
  {throw ContractError("actual footprint policy");}
  const double c = std::cos(yaw), s = std::sin(yaw);
  Bounds b{2., -2., 2., -2.}; double area = 0.;
  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto p = footprint[i], q = footprint[(i + 1) % footprint.size()];
    if (!finite(p) || norm(p) > 2.) {throw ContractError("actual footprint vertex");}
    const Vec2 r{c * p.x - s * p.y, s * p.x + c * p.y};
    b.xmin = std::min(b.xmin, r.x); b.xmax = std::max(b.xmax, r.x);
    b.ymin = std::min(b.ymin, r.y); b.ymax = std::max(b.ymax, r.y);
    area += p.x * q.y - p.y * q.x;
  }
  if (!std::isfinite(area) || std::abs(area) < 1e-6 || b.xmin >= b.xmax || b.ymin >= b.ymax) {
    throw ContractError("degenerate footprint");
  }
  b.xmin -= padding; b.xmax += padding;
  b.ymin -= padding; b.ymax += padding;
  return b;
}
std::string BodyPolicy::digest() const
{
  support(); Digest d; d.text("r4_body/v1"); d.integer(footprint.size());
  for (auto p : footprint) {d.point(p);}
  d.number(padding); d.number(yaw); d.number(static_clearance); return d.finish();
}
std::string BodyPolicy::geometry_digest() const
{
  support(); Digest d; d.text("r4_body_geometry/v2"); d.integer(footprint.size());
  for (auto p : footprint) {d.point(p);}
  d.number(padding); d.number(static_clearance); return d.finish();
}
Bounds BodyPolicy::support_at(double angle) const
{
  auto query = *this; query.yaw = angle; return query.support();
}
void ConsumptionPolicy::validate() const
{
  if (prediction_ttl_ns <= 0 || prediction_ttl_ns > 400000000 ||
    observation_ttl_ns <= 0 || observation_ttl_ns > 400000000 ||
    max_tracks == 0 || max_tracks > 64 || max_members == 0 || max_members > 4096 ||
    max_cells_per_track == 0 || max_cells_per_track > 512 || raster_resolution != 0.05)
  {throw ContractError("prediction budget/TTL/grid policy");}
  for (double value : {geometric_margin, motion_error_speed, halo, slope, residual_scale}) {
    if (!std::isfinite(value) || value < 0) {throw ContractError("soft field policy");}
  }
  if (geometric_margin > 0.5 || motion_error_speed > 3 || halo <= 0 || halo > 1 ||
    slope <= 0 || slope > 100 || residual_scale <= 0 || residual_scale > 100)
  {throw ContractError("soft field policy range");}
}
std::string ConsumptionPolicy::digest() const
{
  validate(); Digest d; d.text("r4_consumption/v1");
  d.integer(prediction_ttl_ns); d.integer(observation_ttl_ns); d.integer(max_tracks);
  d.integer(max_members); d.integer(max_cells_per_track);
  for (double v : {raster_resolution, geometric_margin, motion_error_speed, halo, slope, residual_scale}) {
    d.number(v);
  }
  return d.finish();
}

PredictionSnapshot PredictionSnapshot::freeze(
  const rm_r4_interfaces::msg::ObservedPredictionEnvelope & envelope,
  int64_t epoch_ns, const std::string & expected_frame,
  const BodyPolicy & body, const ConsumptionPolicy & policy)
{
  policy.validate(); const auto support = body.support();
  const auto & p = envelope.prediction;
  const int64_t source = epoch(p.header.stamp);
  if (epoch_ns <= 0 || epoch_ns > std::numeric_limits<int64_t>::max() - 1500000000 ||
    source <= 0 || source > epoch_ns || epoch_ns - source > policy.prediction_ttl_ns ||
    envelope.schema != envelope.SCHEMA || !envelope.complete || envelope.reason != "ok" ||
    envelope.producer_id.empty() || envelope.producer_id.size() > 128 || envelope.sequence == 0 ||
    expected_frame.empty() || p.header.frame_id != expected_frame ||
    p.schema != p.SCHEMA_OBSERVATION_ANCHOR || p.authority != p.AUTHORITY_SHADOW_ONLY ||
    !p.complete || p.prediction_steps != 15 || p.prediction_dt != 0.1 ||
    p.tracks.size() > policy.max_tracks || p.total_track_count != p.tracks.size() ||
    envelope.tracks.size() != p.tracks.size())
  {throw ContractError("atomic prediction contract/frame/TTL/budget");}
  PredictionSnapshot out; out.epoch_ns_ = epoch_ns; out.source_ns_ = source;
  out.frame_ = expected_frame; out.producer_id_ = envelope.producer_id;
  out.generation_ = envelope.producer_generation; out.sequence_ = envelope.sequence;
  out.body_support_ = support; out.body_ = body; out.policy_ = policy;
  out.body_digest_ = body.digest();
  Digest pd; pd.text(out.body_digest_); pd.text(policy.digest()); out.policy_digest_ = pd.finish();
  Digest d; d.text(envelope.schema); d.text(envelope.producer_id);
  d.integer(envelope.producer_generation); d.integer(envelope.sequence);
  d.integer(envelope.complete); d.text(envelope.reason); d.text(p.header.frame_id);
  d.integer(source); d.text(p.schema); d.text(p.authority); d.integer(epoch(p.processing_stamp));
  d.number(p.prediction_dt); d.integer(p.prediction_steps); d.integer(p.complete);
  d.integer(p.total_track_count); d.integer(p.tracks.size());
  std::map<uint64_t, const rm_r4_interfaces::msg::ObservedTrackMembers *> members;
  for (const auto & m : envelope.tracks) {
    if (!members.emplace(m.track_id, &m).second) {throw ContractError("duplicate member track");}
  }
  std::set<uint64_t> ids; size_t total_members = 0;
  for (const auto & t : p.tracks) {
    const int64_t observed = epoch(t.last_observation_stamp);
    const auto anchor = planar(t.position), velocity = planar(t.velocity), size = planar(t.size);
    if (!ids.insert(t.track_id).second || members.count(t.track_id) != 1 ||
      observed <= 0 || observed > source || epoch_ns - observed > policy.observation_ttl_ns ||
      t.observation_count == 0 || norm(velocity) > 3 || size.x <= 0 || size.y <= 0 ||
      size.x > 3 || size.y > 3 || t.prediction.size() != 15 ||
      (t.state != t.STATE_TENTATIVE && t.state != t.STATE_CONFIRMED && t.state != t.STATE_COASTING) ||
      (t.state == t.STATE_CONFIRMED && (observed != source || t.miss_count != 0)) ||
      (t.state == t.STATE_COASTING && (observed >= source || t.miss_count == 0)))
    {throw ContractError("public track identity/lifecycle/observation/CV");}
    d.integer(t.track_id); d.integer(t.state); d.point(anchor); d.number(t.position.z); d.point(velocity); d.number(t.velocity.z);
    d.point(size); d.number(t.size.z);
    d.integer(observed); d.integer(t.observation_count); d.integer(t.miss_count);
    d.integer(t.prediction.size());
    for (size_t i = 0; i < t.prediction.size(); ++i) {
      const auto a = planar(t.prediction[i]); const double dt = (i + 1) * 0.1;
      if (norm(subtract(a, {anchor.x + dt * velocity.x, anchor.y + dt * velocity.y})) > 1e-6) {
        throw ContractError("public v2 display/CV mismatch");
      }
      d.point(a);
    }
    const auto & m = *members.at(t.track_id);
    const auto centroid = planar(m.centroid_at_observation);
    const double age = (source - observed) * 1e-9;
    if (epoch(m.last_observation_stamp) != observed || m.association_sequence == 0 ||
      m.association_sequence > envelope.sequence || m.detection_index >= 4096 ||
      (observed == source && m.association_sequence != envelope.sequence) ||
      (observed < source && m.association_sequence >= envelope.sequence) ||
      m.local_endpoints.empty() || m.local_endpoints.size() != m.source_member_ids.size() ||
      m.local_endpoints.size() > policy.max_members ||
      norm(subtract(anchor, {centroid.x + age * velocity.x, centroid.y + age * velocity.y})) > 1e-6)
    {throw ContractError("same-assignment observed support identity");}
    total_members += m.local_endpoints.size();
    if (total_members > policy.max_members) {throw ContractError("total member budget");}
    d.integer(m.track_id); d.integer(observed); d.integer(m.association_sequence);
    d.integer(m.detection_index); d.point(centroid); d.number(m.centroid_at_observation.z); d.integer(m.local_endpoints.size());
    std::set<uint32_t> beams; std::set<std::pair<int, int>> cells; Vec2 sum{};
    for (size_t i = 0; i < m.local_endpoints.size(); ++i) {
      const auto local = planar(m.local_endpoints[i]);
      if (std::abs(local.x) > 3 || std::abs(local.y) > 3 ||
        !beams.insert(m.source_member_ids[i]).second) {throw ContractError("observed member content");}
      sum.x += local.x; sum.y += local.y;
      cells.emplace(static_cast<int>(std::floor(local.x / policy.raster_resolution)),
        static_cast<int>(std::floor(local.y / policy.raster_resolution)));
      d.point(local); d.number(m.local_endpoints[i].z); d.integer(m.source_member_ids[i]);
    }
    if (norm(sum) / m.local_endpoints.size() > 1e-6 || cells.size() > policy.max_cells_per_track) {
      throw ContractError("centroid/raster budget");
    }
    ObservedRaster raster; raster.track_id = t.track_id; raster.observation_ns = observed;
    raster.source_anchor = anchor;
    raster.state = t.state == t.STATE_TENTATIVE ? "tentative" :
      (t.state == t.STATE_CONFIRMED ? "confirmed" : "coasting");
    raster.velocity = t.state == t.STATE_TENTATIVE ? Vec2{} : velocity;
    for (auto [x, y] : cells) {
      raster.local_cell_centres.push_back({(x + 0.5) * policy.raster_resolution,
        (y + 0.5) * policy.raster_resolution});
    }
    out.tracks_.push_back(std::move(raster));
  }
  // Preserve the original member-record ordering in content identity too.
  for (const auto & m : envelope.tracks) {d.integer(m.track_id);}
  out.receipt_digest_ = d.finish(); return out;
}
int64_t PredictionSnapshot::stage_epoch(size_t stage) const
{
  if (stage > 30) {throw ContractError("stage grid");}
  return epoch_ns_ + static_cast<int64_t>(stage) * 50000000;
}
void ReceiptGate::reset() {last_.reset(); usable_ = false;}
ConsumptionInput ReceiptGate::consume(
  const rm_r4_interfaces::msg::ObservedPredictionEnvelope & envelope,
  int64_t epoch_ns, const std::string & frame, const BodyPolicy & body,
  const ConsumptionPolicy & policy)
{
  try {
    auto snapshot = PredictionSnapshot::freeze(envelope, epoch_ns, frame, body, policy);
    const bool reset_warm = accept(snapshot);
    return {std::move(snapshot), reset_warm};
  } catch (...) {usable_ = false; throw;}
}
bool ReceiptGate::accept(const PredictionSnapshot & s)
{
  const bool was_usable = usable_; usable_ = false;
  bool reset_warm = !was_usable;
  if (last_) {
    const auto & old = *last_;
    if (s.epoch_ns() <= old.epoch_ns()) {throw ContractError("control clock order requires reset");}
    if (s.producer_id() != old.producer_id()) {throw ContractError("producer switch requires reset");}
    if (s.generation() < old.generation()) {throw ContractError("producer generation regression");}
    if (s.generation() == old.generation()) {
      if (s.source_ns() == old.source_ns() && s.sequence() == old.sequence()) {
        if (!was_usable || s.receipt_digest() != old.receipt_digest()) {
          throw ContractError("same receipt invalidated or mutated");
        }
      } else {
        if (s.source_ns() <= old.source_ns() || s.sequence() <= old.sequence()) {
          throw ContractError("prediction receipt order");
        }
        const double allowance = 0.5 + 3 * (s.source_ns() - old.source_ns()) * 1e-9;
        for (const auto & t : s.tracks()) {
          for (const auto & previous : old.tracks()) {
            if (t.track_id == previous.track_id &&
              norm(subtract(t.source_anchor, previous.source_anchor)) > allowance) {
              throw ContractError("public track jump");
            }
          }
        }
      }
    } else {reset_warm = true;}
    reset_warm = reset_warm || s.policy_digest() != old.policy_digest();
  }
  last_ = s; usable_ = true; return reset_warm;
}
TemporalSoftField::TemporalSoftField(PredictionSnapshot snapshot) : snapshot_(std::move(snapshot)) {}
std::vector<Vec2> TemporalSoftField::translated_cells(size_t track, size_t stage) const
{
  const auto stamp = snapshot_.stage_epoch(stage);
  if (track >= snapshot_.tracks().size()) {throw ContractError("track index");}
  const auto & t = snapshot_.tracks()[track]; const double dt = (stamp - snapshot_.source_ns()) * 1e-9;
  std::vector<Vec2> out; out.reserve(t.local_cell_centres.size());
  for (auto local : t.local_cell_centres) {
    out.push_back({local.x + t.source_anchor.x + dt * t.velocity.x,
      local.y + t.source_anchor.y + dt * t.velocity.y});
  }
  return out;
}
SoftSample TemporalSoftField::sample(Vec2 position, size_t stage) const
{
  return sample_support(position, stage, snapshot_.body_support());
}
SoftSample TemporalSoftField::sample(Vec2 position, double yaw, size_t stage) const
{
  return sample_support(position, stage, snapshot_.body_policy().support_at(yaw));
}
SoftSample TemporalSoftField::sample_support(Vec2 position, size_t stage, Bounds support) const
{
  const auto stamp = snapshot_.stage_epoch(stage);
  if (!finite(position)) {throw ContractError("soft query");}
  SoftSample best; const auto & cfg = snapshot_.policy(); const auto b = support;
  for (size_t i = 0; i < snapshot_.tracks().size(); ++i) {
    const auto & t = snapshot_.tracks()[i];
    const double extra = cfg.raster_resolution / 2 + cfg.geometric_margin +
      cfg.motion_error_speed * (stamp - t.observation_ns) * 1e-9;
    const Vec2 mid{-(b.xmin + b.xmax) / 2, -(b.ymin + b.ymax) / 2};
    const Vec2 half{(b.xmax - b.xmin) / 2 + extra, (b.ymax - b.ymin) / 2 + extra};
    double minimum = std::numeric_limits<double>::infinity(); Vec2 normal{};
    for (auto cell : translated_cells(i, stage)) {
      const Vec2 offset{position.x - cell.x - mid.x, position.y - cell.y - mid.y};
      const Vec2 d{std::abs(offset.x) - half.x, std::abs(offset.y) - half.y};
      const Vec2 outside{std::max(d.x, 0.), std::max(d.y, 0.)};
      const double length = norm(outside);
      const double distance = length + std::min(std::max(d.x, d.y), 0.);
      if (distance < minimum) {
        minimum = distance; normal = {};
        if (distance > 0.) {
          normal = {outside.x / length * (offset.x < 0 ? -1 : 1),
            outside.y / length * (offset.y < 0 ? -1 : 1)};
        }
      }
    }
    if (minimum >= cfg.halo) {continue;}
    const double residual = cfg.residual_scale * std::exp(-cfg.slope * std::max(minimum, 0.));
    if (residual > best.residual) {
      best = {residual, {-cfg.slope * residual * normal.x, -cfg.slope * residual * normal.y},
        minimum, t.track_id, minimum <= 0.};
    }
  }
  return best;
}
FollowResidual free_progress_residual(
  const PreparedCorridor & route, const TemporalSoftField & field, Vec2 xy,
  Vec2 world_velocity, double progress, double progress_rate, size_t stage, double cruise_speed)
{
  if (route.frame() != field.frame() || route.body_digest() != field.body_digest() ||
    !finite(xy) || !finite(world_velocity) || !std::isfinite(progress) ||
    progress < 0 || progress > route.arcs().back() || !std::isfinite(progress_rate) ||
    progress_rate < 0 || progress_rate > 3 || !std::isfinite(cruise_speed) ||
    cruise_speed <= 0 || cruise_speed > 3 || stage > 30) {throw ContractError("Follow residual query");}
  if (stage == 30) {return {};}
  const auto ref = route.sample(progress); const auto soft = field.sample(xy, stage);
  const Vec2 normal{-ref.tangent.y, ref.tangent.x}, delta = subtract(xy, ref.position);
  const double cruise = std::min(cruise_speed, std::sqrt(2 * std::max(0., route.arcs().back() - progress)));
  FollowResidual r; r.contour = dot(normal, delta); r.lag = dot(ref.tangent, delta);
  r.projection = dot(ref.tangent, world_velocity) - progress_rate;
  r.cruise = progress_rate - cruise; r.dynamic = soft.residual;
  constexpr double dt = 0.05;
  r.cost = 0.5 * dt * (20 * r.contour * r.contour + 8 * r.lag * r.lag +
    2 * r.projection * r.projection + 2 * r.cruise * r.cruise + r.dynamic * r.dynamic) -
    0.1 * dt * progress_rate;
  r.xy_gradient = {dt * (20 * r.contour * normal.x + 8 * r.lag * ref.tangent.x +
    r.dynamic * soft.gradient.x), dt * (20 * r.contour * normal.y + 8 * r.lag * ref.tangent.y +
    r.dynamic * soft.gradient.y)};
  r.world_velocity_gradient = {dt * 2 * r.projection * ref.tangent.x,
    dt * 2 * r.projection * ref.tangent.y};
  // Like the A02 local QP, tangent and cruise reference are frozen for this
  // linearization. Curved-path / cruise-reference derivatives are not claimed.
  r.progress_gradient = -dt * 8 * r.lag;
  r.progress_rate_gradient = dt * (-2 * r.projection + 2 * r.cruise - 0.1);
  return r;
}
}  // namespace rm_r4_prediction_consumption
