// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rm_tdt_planner
{
struct Point {double x = 0.0; double y = 0.0;};

// Nav2 cost bytes, row-major, y increasing upwards. Unknown (255) is blocked.
struct Grid
{
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<uint8_t> costs;
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

// No ROS, global state, cached map, hardware IO, or velocity output.
// Returned samples are a geometric polyline, not a timed trajectory.
Result plan(const Grid & grid, Point start, Point goal, const Options & options);
// Checks every segment against conservative configuration-space occupancy.
bool collision_free(const Grid & grid, const std::vector<Point> & path,
  const Options & options);
}  // namespace rm_tdt_planner
