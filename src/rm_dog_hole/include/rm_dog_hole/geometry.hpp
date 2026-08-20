#pragma once

#include <utility>
#include <vector>

namespace rm_dog_hole
{

using Footprint = std::vector<std::pair<double, double>>;

struct Corridor
{
  double center_x = 0.0;
  double center_y = 0.0;
  double yaw = 0.0;
  double width = 0.8;
  double length = 1.0;
  double traversal_yaw_offset = 0.0;
};

struct CorridorPose
{
  double longitudinal = 0.0;
  double lateral = 0.0;
  double heading_error = 0.0;
  double minimum_wall_clearance = 0.0;
};

double normalizeAngle(double angle);

CorridorPose evaluatePose(
  double x,
  double y,
  double yaw,
  const Corridor & corridor,
  double robot_length,
  double robot_width);

CorridorPose evaluatePose(
  double x,
  double y,
  double yaw,
  const Corridor & corridor,
  const Footprint & footprint);

double projectedHalfWidth(
  const Footprint & footprint,
  double base_yaw,
  double corridor_yaw);

double projectedHalfLength(
  const Footprint & footprint,
  double base_yaw,
  double corridor_yaw);

bool pointInTraversalZone(
  double x,
  double y,
  const Corridor & corridor,
  double entry_margin,
  double exit_margin);

bool pathCrossesCorridor(
  const std::vector<std::pair<double, double>> & points,
  const Corridor & corridor,
  double entry_margin,
  double exit_margin);

std::pair<double, double> pointAtLongitudinal(
  const Corridor & corridor,
  double longitudinal);

double requiredAlignmentOffset(
  double robot_length,
  double deck_height,
  double entry_slope_rad,
  double safety_margin);

}  // namespace rm_dog_hole
