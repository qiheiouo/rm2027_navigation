// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#pragma once
#include "rm_tdt_planner/planner.hpp"
#include <cstddef>
#include <optional>

namespace rm_tdt_planner::pose_geometry
{
// Explicit raw Nav2 cost bytes only. No Grid/PreparedGrid conversion is provided:
// callers must identify the raw source, not pass an inflated configuration mask.
struct RawCostmapInput
{
  int width = 0, height = 0;
  double resolution = 0.0, origin_x = 0.0, origin_y = 0.0;
  std::vector<uint8_t> costs;
};
struct Cell {int x, y; uint8_t cost; bool boundary;};
class RawSnapshot
{
public:
  // Takes an owned copy (or moved input); invalid inputs throw invalid_argument.
  explicit RawSnapshot(RawCostmapInput input);
  const RawCostmapInput & raw() const {return raw_;}
  const std::vector<Cell> & blocked_cells() const {return blocked_;}
private:
  const RawCostmapInput raw_;
  const std::vector<Cell> blocked_;
};
struct Pose {double x = 0.0, y = 0.0, yaw = 0.0;};
struct Motion
{
  Pose start, goal;
  // Explicit signed rotation, including long rotations. Must end at goal.yaw mod 2*pi.
  double yaw_delta = 0.0;
};
struct Limits
{
  double clearance = 0.02;
  double time_budget_seconds = 1.0;
  size_t max_intervals = 8191;
  size_t max_cell_checks = 2000000;
  unsigned int max_depth = 20;
};
enum class Status {Certified, Collision, Unresolved};
enum class Constraint {Footprint, InscribedCentre, MapExtent};
struct Witness
{
  Constraint constraint = Constraint::Footprint;
  int cell_x = -1, cell_y = -1;  // -1 for a centre outside map extent.
  uint8_t cost = 0;
  bool boundary = false;
  Pose pose;  // World pose at this parameter, yaw unwrapped according to Motion.
  double parameter = 0.0;
  double distance = 0.0, required = 0.0;
};
struct Interval
{
  double begin = 0.0, end = 0.0;
  // Lower bound on distance minus required clearance for ALL constraints,
  // including centre-only 253 (required=0). Not a physical body clearance.
  double constraint_margin_lower_bound = 0.0;
};
struct Result
{
  Status status = Status::Unresolved;
  std::string reason;
  std::optional<Witness> witness;
  std::vector<Interval> intervals;  // Complete [0,1] cover only when Certified.
  size_t intervals_examined = 0, cell_checks = 0;
  double elapsed_seconds = 0.0;
};
// Offline only: certify linear centre motion and linear EXPLICIT yaw rotation.
// Convex footprint is already padded, must contain base origin strictly inside.
// No path generation, controller invocation, ROS IO, or arbitrary-rotation guarantee.
Result certify(const RawSnapshot & snapshot, const std::vector<Point> & padded_footprint,
  const Motion & motion, const Limits & limits = Limits{});
}  // namespace rm_tdt_planner::pose_geometry
