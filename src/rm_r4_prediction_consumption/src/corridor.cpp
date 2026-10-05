#include "rm_r4_prediction_consumption/consumption.hpp"
#include "digest.hpp"
#include "sfcSquare.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm_r4_prediction_consumption
{
namespace
{
bool finite(Vec2 p) {return std::isfinite(p.x) && std::isfinite(p.y);}
double distance(Vec2 a, Vec2 b) {return std::hypot(a.x - b.x, a.y - b.y);}
bool inside(Vec2 p, Bounds b)
{
  return p.x > b.xmin && p.x < b.xmax && p.y > b.ymin && p.y < b.ymax;
}
// Independent, closed-square check of the whole static mechanical support.
// The Sfc rectangle itself does not certify continuous occupancy or boundaries.
void certify(const nav_msgs::msg::OccupancyGrid & grid, Bounds support)
{
  const double res = grid.info.resolution, ox = grid.info.origin.position.x,
    oy = grid.info.origin.position.y;
  constexpr double guard = 1e-6;
  if (!(support.xmin > ox + guard && support.ymin > oy + guard &&
    support.xmax < ox + grid.info.width * res - guard &&
    support.ymax < oy + grid.info.height * res - guard)) {throw ContractError("static support outside map");}
  const int left = static_cast<int>(std::floor((support.xmin - ox - guard) / res));
  const int right = static_cast<int>(std::floor((support.xmax - ox + guard) / res));
  const int bottom = static_cast<int>(std::floor((support.ymin - oy - guard) / res));
  const int top = static_cast<int>(std::floor((support.ymax - oy + guard) / res));
  for (int y = bottom; y <= top; ++y) {
    for (int x = left; x <= right; ++x) {
      if (grid.data[static_cast<size_t>(y) * grid.info.width + x] != 0) {
        throw ContractError("Sfc rectangle lacks raw-static support");
      }
    }
  }
}
}
std::string PreparedCorridor::fingerprint_path(const nav_msgs::msg::Path & path)
{
  if (path.header.frame_id.empty() || path.header.frame_id.size() > 128 ||
    path.poses.size() < 2 || path.poses.size() > 512) {throw ContractError("path identity extent");}
  Digest hash; hash.text("r4_path/v1"); hash.text(path.header.frame_id);
  hash.integer(path.header.stamp.sec); hash.integer(path.header.stamp.nanosec); hash.integer(path.poses.size());
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position; const auto & q = pose.pose.orientation;
    if (pose.header.frame_id.size() > 128 || !finite({p.x, p.y}) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {throw ContractError("path identity values");}
    hash.text(pose.header.frame_id); hash.integer(pose.header.stamp.sec); hash.integer(pose.header.stamp.nanosec);
    hash.number(p.x); hash.number(p.y); hash.number(p.z);
    hash.number(q.x); hash.number(q.y); hash.number(q.z); hash.number(q.w);
  }
  return hash.finish();
}
PreparedCorridor PreparedCorridor::prepare(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & grid,
  const BodyPolicy & body, uint64_t generation, double max_range)
{
  const auto support = body.support(); const auto & info = grid.info;
  const auto & q = info.origin.orientation; const auto & origin = info.origin.position;
  if (path.header.frame_id.empty() || path.header.frame_id != grid.header.frame_id ||
    path.poses.size() < 2 || path.poses.size() > 512 || info.width < 8 || info.height < 8 ||
    info.width > 2048 || info.height > 2048 ||
    grid.data.size() != static_cast<size_t>(info.width) * info.height ||
    !std::isfinite(info.resolution) || info.resolution < 0.01 || info.resolution > 1.0 ||
    !finite({origin.x, origin.y}) || !std::isfinite(origin.z) || origin.z != 0. ||
    q.x != 0. || q.y != 0. || q.z != 0. || !std::isfinite(q.w) || std::abs(q.w) != 1. ||
    !std::isfinite(max_range) || max_range < 0.1 || max_range > 5.)
  {throw ContractError("raw-static path/map/frame policy");}
  PreparedCorridor out; out.frame_ = path.header.frame_id; out.generation_ = generation;
  Digest map_hash, policy_hash;
  const auto path_digest = fingerprint_path(path);
  const double res = info.resolution;
  // Black halo protects the vendor's neighbour accesses. Feed cell-centre local
  // coordinates with zero origin; never use its inconsistent nonzero-origin API.
  constexpr int pad = 4;
  const int width = static_cast<int>(info.width) + 2 * pad;
  const int height = static_cast<int>(info.height) + 2 * pad;
  std::vector<unsigned char> mask(static_cast<size_t>(width) * height, 0);
  map_hash.text("r4_raw_static/v1"); map_hash.text(grid.header.frame_id);
  map_hash.integer(info.width); map_hash.integer(info.height); map_hash.number(res);
  map_hash.number(origin.x); map_hash.number(origin.y); map_hash.number(origin.z);
  map_hash.number(q.x); map_hash.number(q.y); map_hash.number(q.z); map_hash.number(q.w);
  for (size_t i = 0; i < grid.data.size(); ++i) {
    const auto value = grid.data[i];
    if (value < -1 || value > 100) {throw ContractError("raw occupancy interpretation");}
    map_hash.integer(static_cast<uint64_t>(static_cast<int64_t>(value)));
    const int x = i % info.width, y = i / info.width;
    mask[static_cast<size_t>(y + pad) * width + x + pad] = value == 0 ? 255 : 0;
  }
  std::vector<Eigen::Vector2f> local;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i]; const auto & p = pose.pose.position;
    const auto & rotation = pose.pose.orientation;
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != out.frame_) ||
      !finite({p.x, p.y}) || p.z != 0. || !std::isfinite(rotation.x) ||
      !std::isfinite(rotation.y) || !std::isfinite(rotation.z) || !std::isfinite(rotation.w) ||
      p.x < origin.x || p.y < origin.y || p.x >= origin.x + info.width * res ||
      p.y >= origin.y + info.height * res) {throw ContractError("path point/frame/outside map");}
    const int x = std::floor((p.x - origin.x) / res), y = std::floor((p.y - origin.y) / res);
    if (grid.data[static_cast<size_t>(y) * info.width + x] != 0) {
      throw ContractError("path anchor not raw-static free");
    }
    if (!out.points_.empty() && distance(out.points_.back(), {p.x, p.y}) <= 1e-9) {continue;}
    out.indices_.push_back(i); out.points_.push_back({p.x, p.y});
    out.arcs_.push_back(out.points_.size() == 1 ? 0. : out.arcs_.back() +
      distance(out.points_[out.points_.size() - 2], out.points_.back()));
    local.emplace_back(p.x - origin.x + (pad - 0.5) * res,
      p.y - origin.y + (pad - 0.5) * res);
  }
  if (out.points_.size() < 2) {throw ContractError("degenerate path");}
  // getCorridor pins its first/last bounds. Duplicate the ends solely for this
  // call so its existing interior API supplies bounds for every original point.
  local.insert(local.begin(), local.front()); local.push_back(local.back());
  SfcSquare provider(width, height, mask.data(), static_cast<float>(res));
  const auto result = provider.getCorridor(local, static_cast<float>(max_range), 0.f);
  if (result.corridor.size() != local.size() || result.index.size() != local.size()) {
    throw ContractError("Sfc output coverage");
  }
  for (size_t i = 0; i < out.points_.size(); ++i) {
    if (result.index[i + 1] != static_cast<int>(i + 1)) {throw ContractError("Sfc path index identity");}
    const auto & raw = result.corridor[i + 1];
    // Vendor bounds refer to extreme cell centres. Retaining those centres
    // rather than enlarging to cell edges is deliberately conservative.
    Bounds b{raw[0] + origin.x - (pad - 0.5) * res - support.xmin + body.static_clearance,
      raw[2] + origin.x - (pad - 0.5) * res - support.xmax - body.static_clearance,
      raw[1] + origin.y - (pad - 0.5) * res - support.ymin + body.static_clearance,
      raw[3] + origin.y - (pad - 0.5) * res - support.ymax - body.static_clearance};
    if (!std::isfinite(b.xmin) || !std::isfinite(b.xmax) || !std::isfinite(b.ymin) ||
      !std::isfinite(b.ymax) || b.xmin >= b.xmax || b.ymin >= b.ymax || !inside(out.points_[i], b))
    {throw ContractError("Sfc degenerate or anchor outside centre bounds");}
    certify(grid, {b.xmin + support.xmin - body.static_clearance,
      b.xmax + support.xmax + body.static_clearance,
      b.ymin + support.ymin - body.static_clearance,
      b.ymax + support.ymax + body.static_clearance});
    out.bounds_.push_back(b);
  }
  // Every original line segment must be covered by its neighbouring certified
  // convex rectangles. No resampling, repair or second path search is performed.
  auto interval = [](Vec2 a, Vec2 b, Bounds box) {
      double lower = 0., upper = 1.;
      auto axis = [&](double p, double delta, double min, double max) {
          if (std::abs(delta) < 1e-12) {
            if (p < min || p > max) {lower = 1.; upper = 0.;}
          } else {
            const double left = (min - p) / delta, right = (max - p) / delta;
            lower = std::max(lower, std::min(left, right));
            upper = std::min(upper, std::max(left, right));
          }
        };
      axis(a.x, b.x - a.x, box.xmin, box.xmax);
      axis(a.y, b.y - a.y, box.ymin, box.ymax);
      return std::pair<double, double>{lower, upper};
    };
  for (size_t i = 0; i + 1 < out.points_.size(); ++i) {
    const auto first = interval(out.points_[i], out.points_[i + 1], out.bounds_[i]);
    const auto next = interval(out.points_[i], out.points_[i + 1], out.bounds_[i + 1]);
    if (first.first > 0. || next.second < 1. || first.second < next.first) {
      throw ContractError("path segment lacks continuous static corridor support");
    }
  }
  out.path_digest_ = path_digest; out.map_digest_ = map_hash.finish();
  out.body_digest_ = body.digest();
  policy_hash.text(out.body_digest_); policy_hash.number(max_range); out.policy_digest_ = policy_hash.finish();
  return out;
}
PathSample PreparedCorridor::sample(double progress) const
{
  if (!std::isfinite(progress) || progress < 0 || progress > arcs_.back()) {throw ContractError("path progress");}
  const auto upper = std::upper_bound(arcs_.begin(), arcs_.end(), progress);
  const size_t index = std::min(static_cast<size_t>(upper - arcs_.begin() - 1), points_.size() - 2);
  const double length = arcs_[index + 1] - arcs_[index];
  const Vec2 tangent{(points_[index + 1].x - points_[index].x) / length,
    (points_[index + 1].y - points_[index].y) / length};
  return {{points_[index].x + (progress - arcs_[index]) * tangent.x,
      points_[index].y + (progress - arcs_[index]) * tangent.y}, tangent};
}
double PreparedCorridor::project(Vec2 position) const
{
  if (!finite(position)) {throw ContractError("path projection");}
  double best = std::numeric_limits<double>::infinity(), progress = 0.;
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    const auto a = points_[i], b = points_[i + 1]; const Vec2 edge{b.x - a.x, b.y - a.y};
    const double fraction = std::clamp(((position.x - a.x) * edge.x +
      (position.y - a.y) * edge.y) / (edge.x * edge.x + edge.y * edge.y), 0., 1.);
    const double error = distance(position, {a.x + fraction * edge.x, a.y + fraction * edge.y});
    if (error < best) {best = error; progress = arcs_[i] + fraction * (arcs_[i + 1] - arcs_[i]);}
  }
  return progress;
}
Bounds PreparedCorridor::local_bounds(Vec2 position) const
{
  if (!finite(position)) {throw ContractError("local corridor query");}
  std::optional<size_t> selected; double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < bounds_.size(); ++i) {
    if (inside(position, bounds_[i])) {
      const double error = distance(position, points_[i]);
      if (error < best) {selected = i; best = error;}
    }
  }
  if (!selected) {throw ContractError("state outside certified static corridor");}
  return bounds_[*selected];
}
}  // namespace rm_r4_prediction_consumption
