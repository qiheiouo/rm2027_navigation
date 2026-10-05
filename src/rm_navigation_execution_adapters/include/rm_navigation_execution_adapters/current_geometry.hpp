#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rm_navigation_execution_adapters
{
struct Point {double x{}, y{};};
struct Pose {double x{}, y{}, yaw{};};
struct Twist {double vx{}, vy{}, wz{};};
struct GridMetadata
{
  std::string frame, revision;
  int64_t update_ns{}, receipt_ns{};
  bool current{};
};
struct RawCurrentGrid
{
  int width{}, height{};
  double resolution{}, origin_x{}, origin_y{};
  std::vector<uint8_t> costs;  // Nav2 master: 253 centre, >=254 filled footprint.
  GridMetadata metadata;
};
class CurrentGridSnapshot
{
public:
  // Preparation/copy belongs to the existing costmap update boundary, not a
  // new map pipeline or a solver/TF call under the original map lock.
  explicit CurrentGridSnapshot(RawCurrentGrid grid);
  const GridMetadata & metadata() const;
private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
  friend struct GeometryAccess;
};
struct Body
{
  std::vector<Point> padded_footprint;  // Existing actual convex padded footprint.
  double padding{}, clearance{};
  std::string revision;
};
struct MotionBounds
{
  Point lower, upper;
  double yaw_rate{};  // Explicit bound; command slew is not physical tracking.
  double position_error{}, yaw_error{}, tracking_position_error{}, tracking_yaw_error{};
  std::string revision;
};
struct GeometryPolicy
{
  double budget_seconds{.010};
  size_t max_intervals{255}, max_cell_checks{50000};
  unsigned int max_depth{12};
  int64_t state_ttl_ns{100000000}, update_ttl_ns{150000000}, receipt_ttl_ns{250000000};
};
struct CurrentCommandInput
{
  CurrentGridSnapshot grid;
  Body body;
  MotionBounds bounds;
  Pose pose;
  Twist measured, candidate;  // Candidate already transformed by original owner.
  std::string frame, base_frame, candidate_identity;
  int64_t epoch_ns{}, pose_stamp_ns{}, velocity_stamp_ns{}, tf_stamp_ns{}, hold_ns{};
};
struct CoveredInterval {double begin_seconds{}, end_seconds{}, margin_lower_bound{};};
struct BranchEvidence
{
  double chord_error{}, centre_reserve{}, footprint_reserve{};
  std::vector<CoveredInterval> cover;
};
enum class GeometryStatus {Certified, Collision, Unavailable};
struct GeometryResult
{
  GeometryStatus status{GeometryStatus::Unavailable};
  std::string reason, candidate_identity, map_revision, body_revision;
  std::string frame, base_frame, limits_revision;
  Twist candidate;
  int64_t epoch_ns{}, hold_ns{}, processing_budget_ns{};
  int collision_branch{-1};  // 0 measured-held, 1 candidate-held.
  double elapsed_seconds{};
  size_t cell_checks{}, intervals_examined{};
  std::vector<BranchEvidence> branches;  // Complete covers only when Certified.
};
// Exact constant BODY twist pose, including reverse/lateral/rotating motion.
Pose integrate_held_body_twist(Pose start, Twist velocity, double seconds);
// Geometric model evidence only: no grant, lease, publisher, output, brake or
// future prediction. Host must contain the actual response in declared tubes.
GeometryResult inspect_current_command(
  const CurrentCommandInput & input, const GeometryPolicy & policy = {});
}  // namespace rm_navigation_execution_adapters
