// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rm_tdt_planner
{
struct Point {double x = 0.0; double y = 0.0;};

// Unannotated/offline grids retain the historical conservative seed policy.
// Nav2 master costmaps already encode a footprint-dependent centre exclusion at 253.
enum class CostInterpretation {ObstacleSeeds, Nav2Master};

// Nav2 cost bytes, row-major, y increasing upwards. Unknown (255) is blocked.
struct Grid
{
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<uint8_t> costs;
  CostInterpretation cost_interpretation = CostInterpretation::ObstacleSeeds;
};

struct Options
{
  // Circumscribed footprint radius, including padding. Rotation is unconstrained.
  double radius = 0.45;
  double clearance = 0.02;
  double potential_weight = 0.5;
  double simplify_tolerance = 0.10;
  double output_spacing = 0.025;
  double time_budget = 0.25;
  double nominal_speed = 1.0;
  double nominal_acceleration = 1.0;
  double corridor_range = 1.0;
  int collision_iterations = 3;
  int max_waypoints = 32;
  bool optimize = true;
};

struct Result
{
  bool success = false;
  bool optimized = false;
  std::string reason;
  std::vector<Point> path;
  double elapsed_seconds = 0.0;
};

// Immutable configuration-space snapshot produced only by prepare_grid().
// Its cells already include the physical footprint and clearance. Exposed for
// offline comparisons; deployment plugins cannot bypass preparation by parameter.
class PreparedGrid
{
public:
  const Grid & grid() const {return grid_;}
private:
  PreparedGrid(Grid grid, double radius, double clearance);
  Grid grid_;
  double radius_;
  double clearance_;
  friend PreparedGrid prepare_grid(const Grid &, const Options &);
  friend Result plan_prepared(const PreparedGrid &, Point, Point, const Options &);
};
PreparedGrid prepare_grid(const Grid & grid, const Options & options);
Result plan_prepared(const PreparedGrid & grid, Point start, Point goal,
  const Options & options);
bool collision_free_prepared(const PreparedGrid & grid, const std::vector<Point> & path);

// No ROS, global state, cached map, hardware IO, or velocity output.
// Returned samples are a geometric polyline, not a timed trajectory.
Result plan(const Grid & grid, Point start, Point goal, const Options & options);
// Checks every segment against conservative configuration-space occupancy.
bool collision_free(const Grid & grid, const std::vector<Point> & path,
  const Options & options);
}  // namespace rm_tdt_planner
