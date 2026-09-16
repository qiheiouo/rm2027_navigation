// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#pragma once
#include "rm_tdt_planner/planner.hpp"
namespace rm_tdt_planner
{
// Simulation opt-in only. Preserve every XY point; return unwrapped yaw values.
// Central 0.20 m chord estimates the tangent through polyline corners. Smoothstep
// blends the first 0.40 m from start yaw and final 1.00 m to exact goal yaw;
// each band is capped at half path length. No velocity/rotation safety guarantee.
std::vector<double> continuous_path_heading(const std::vector<Point> & path,
  double start_yaw, double goal_yaw);
}
