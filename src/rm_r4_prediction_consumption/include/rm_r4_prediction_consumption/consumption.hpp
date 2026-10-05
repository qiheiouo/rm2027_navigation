#pragma once

#include <cstdint>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rm_r4_interfaces/msg/observed_prediction_envelope.hpp>

namespace rm_r4_prediction_consumption
{
struct ContractError : std::invalid_argument {using std::invalid_argument::invalid_argument;};
struct Vec2 {double x{}, y{};};
struct Bounds {double xmin{}, xmax{}, ymin{}, ymax{};};
struct BodySupport {Bounds bounds, yaw_derivative;};

// Caller supplies the actual footprint and fixed world yaw, not harness geometry.
// Keep the registered positive padding and at least 0.05 m static clearance.
struct BodyPolicy
{
  std::vector<Vec2> footprint;
  double padding{};
  double yaw{};
  double static_clearance{};
  Bounds support() const;
  BodySupport support_at(double query_yaw) const;
  Bounds swept_support(double begin_yaw, double end_yaw) const;
  std::string geometry_digest() const;
  std::string digest() const;
};

struct ConsumptionPolicy
{
  int64_t prediction_ttl_ns{400000000}, observation_ttl_ns{400000000};
  size_t max_tracks{4}, max_members{4096}, max_cells_per_track{512};
  double raster_resolution{0.05}, geometric_margin{0.02};
  double motion_error_speed{}, halo{0.40}, slope{16.0}, residual_scale{8.0};
  void validate() const;
  std::string digest() const;
};

struct ObservedRaster
{
  uint64_t track_id{};
  std::string state;
  int64_t observation_ns{};
  Vec2 source_anchor, velocity;
  std::vector<Vec2> local_cell_centres;
};

class PredictionSnapshot
{
public:
  static PredictionSnapshot freeze(
    const rm_r4_interfaces::msg::ObservedPredictionEnvelope & envelope,
    int64_t epoch_ns, const std::string & expected_frame,
    const BodyPolicy & body, const ConsumptionPolicy & policy = {});
  int64_t epoch_ns() const {return epoch_ns_;}
  int64_t source_ns() const {return source_ns_;}
  int64_t stage_epoch(size_t stage) const;
  const std::string & frame() const {return frame_;}
  const std::string & producer_id() const {return producer_id_;}
  uint64_t generation() const {return generation_;}
  uint64_t sequence() const {return sequence_;}
  const std::string & receipt_digest() const {return receipt_digest_;}
  const std::string & policy_digest() const {return policy_digest_;}
  const std::string & body_digest() const {return body_digest_;}
  const std::vector<ObservedRaster> & tracks() const {return tracks_;}
  const Bounds & body_support() const {return body_support_;}
  const BodyPolicy & body_policy() const {return body_;}
  const ConsumptionPolicy & policy() const {return policy_;}
private:
  PredictionSnapshot() = default;
  int64_t epoch_ns_{}, source_ns_{};
  uint64_t generation_{}, sequence_{};
  std::string frame_, producer_id_, receipt_digest_, policy_digest_, body_digest_;
  Bounds body_support_;
  BodyPolicy body_;
  ConsumptionPolicy policy_;
  std::vector<ObservedRaster> tracks_;
};

struct ConsumptionInput {PredictionSnapshot snapshot; bool reset_warm;};

// Rejected receipts never restore cached input. Explicit lifecycle reset is
// required for a different producer instance; all invalidations clear warm use.
class ReceiptGate
{
public:
  ConsumptionInput consume(
    const rm_r4_interfaces::msg::ObservedPredictionEnvelope & envelope,
    int64_t epoch_ns, const std::string & expected_frame, const BodyPolicy & body,
    const ConsumptionPolicy & policy = {});
  bool accept(const PredictionSnapshot & snapshot);  // true => reset warm state
  bool usable() const {return usable_;}
  void reset();
private:
  std::optional<PredictionSnapshot> last_;
  bool usable_{false};
};

struct SoftSample
{
  double residual{};
  Vec2 gradient;
  std::optional<double> clearance;
  std::optional<uint64_t> track_id;
  bool plateau{};
  double yaw_gradient{};
};

class TemporalSoftField
{
public:
  explicit TemporalSoftField(PredictionSnapshot snapshot);
  std::vector<Vec2> translated_cells(size_t track, size_t stage) const;
  SoftSample sample(Vec2 position, size_t stage) const;
  SoftSample sample(Vec2 position, double yaw, size_t stage) const;
  const std::string & frame() const {return snapshot_.frame();}
  const std::string & body_digest() const {return snapshot_.body_digest();}
private:
  PredictionSnapshot snapshot_;
  SoftSample sample_support(Vec2 position, size_t stage, BodySupport support) const;
};

struct PathSample {Vec2 position, tangent;};
class PreparedCorridor
{
public:
  static PreparedCorridor prepare(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & raw_static,
    const BodyPolicy & body, uint64_t generation, double max_range = 2.0);
  // Same existing identity encoding, for the original host's setPlan binding.
  static std::string fingerprint_path(const nav_msgs::msg::Path & path);
  const std::vector<Vec2> & points() const {return points_;}
  const std::vector<double> & arcs() const {return arcs_;}
  const std::vector<Bounds> & centre_bounds() const {return bounds_;}
  const std::vector<size_t> & path_indices() const {return indices_;}
  const std::string & frame() const {return frame_;}
  const std::string & path_digest() const {return path_digest_;}
  const std::string & map_digest() const {return map_digest_;}
  const std::string & policy_digest() const {return policy_digest_;}
  const std::string & body_digest() const {return body_digest_;}
  uint64_t generation() const {return generation_;}
  PathSample sample(double progress) const;
  double project(Vec2 position) const;
  Bounds local_bounds(Vec2 position) const;
  // Read-only view of the same certified Sfc rectangles, before body erosion.
  Bounds local_free_bounds(Vec2 position, const BodyPolicy & query_body) const;
  const std::string & rotating_policy_digest() const {return rotating_policy_digest_;}
private:
  PreparedCorridor() = default;
  std::vector<Vec2> points_;
  std::vector<double> arcs_;
  std::vector<Bounds> bounds_;
  std::vector<Bounds> free_bounds_;
  std::vector<size_t> indices_;
  std::string frame_, path_digest_, map_digest_, policy_digest_, body_digest_;
  std::string rotating_policy_digest_;
  uint64_t generation_{};
};

struct FollowResidual
{
  double contour{}, lag{}, projection{}, cruise{}, dynamic{}, cost{};
  Vec2 xy_gradient, world_velocity_gradient;
  double progress_gradient{}, progress_rate_gradient{};
};

// Running-stage objective and local linearization; s_dot is an independent
// decision variable. This supplies mathematics to a later bounded solver.
FollowResidual free_progress_residual(
  const PreparedCorridor & route, const TemporalSoftField & field, Vec2 xy,
  Vec2 world_velocity, double progress, double progress_rate, size_t stage,
  double cruise_speed);
}  // namespace rm_r4_prediction_consumption
