#include "rm_dog_hole/geometry.hpp"

#include <cmath>

namespace rm_dog_hole
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

CorridorPose evaluatePose(
  double x,
  double y,
  double yaw,
  const Corridor & corridor,
  double robot_length,
  double robot_width)
{
  const double axis_x = std::cos(corridor.yaw);
  const double axis_y = std::sin(corridor.yaw);
  const double normal_x = -axis_y;
  const double normal_y = axis_x;
  const double delta_x = x - corridor.center_x;
  const double delta_y = y - corridor.center_y;

  CorridorPose pose;
  pose.longitudinal = delta_x * axis_x + delta_y * axis_y;
  pose.lateral = delta_x * normal_x + delta_y * normal_y;
  pose.heading_error = normalizeAngle(corridor.yaw - yaw);
  const double projected_half_width =
    0.5 * robot_width * std::abs(std::cos(pose.heading_error)) +
    0.5 * robot_length * std::abs(std::sin(pose.heading_error));
  pose.minimum_wall_clearance =
    0.5 * corridor.width - projected_half_width - std::abs(pose.lateral);
  return pose;
}

bool pointInTraversalZone(
  double x,
  double y,
  const Corridor & corridor,
  double entry_margin,
  double exit_margin)
{
  const auto pose = evaluatePose(x, y, corridor.yaw, corridor, 0.0, 0.0);
  const double minimum_longitudinal = -0.5 * corridor.length - entry_margin;
  const double maximum_longitudinal = 0.5 * corridor.length + exit_margin;
  return pose.longitudinal >= minimum_longitudinal &&
         pose.longitudinal <= maximum_longitudinal &&
         std::abs(pose.lateral) <= 0.5 * corridor.width;
}

bool pathCrossesCorridor(
  const std::vector<std::pair<double, double>> & points,
  const Corridor & corridor,
  double entry_margin,
  double exit_margin)
{
  bool saw_entry_side = false;
  bool saw_exit_side = false;
  for (const auto & point : points) {
    if (!pointInTraversalZone(
        point.first, point.second, corridor, entry_margin, exit_margin))
    {
      continue;
    }
    const auto pose = evaluatePose(
      point.first, point.second, corridor.yaw, corridor, 0.0, 0.0);
    saw_entry_side = saw_entry_side || pose.longitudinal <= -0.25 * corridor.length;
    saw_exit_side = saw_exit_side || pose.longitudinal >= 0.25 * corridor.length;
  }
  return saw_entry_side && saw_exit_side;
}

std::pair<double, double> pointAtLongitudinal(
  const Corridor & corridor,
  double longitudinal)
{
  return {
    corridor.center_x + longitudinal * std::cos(corridor.yaw),
    corridor.center_y + longitudinal * std::sin(corridor.yaw)
  };
}

double requiredAlignmentOffset(
  double robot_length,
  double deck_height,
  double entry_slope_rad,
  double safety_margin)
{
  const double ramp_horizontal_length =
    deck_height > 0.0 && entry_slope_rad > 0.0 ?
    deck_height / std::tan(entry_slope_rad) : 0.0;
  return 0.5 * robot_length + ramp_horizontal_length + safety_margin;
}

}  // namespace rm_dog_hole
