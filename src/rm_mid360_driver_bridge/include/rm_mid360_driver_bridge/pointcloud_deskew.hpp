#pragma once

#include <tf2/LinearMath/Transform.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace rm_mid360_driver_bridge
{

struct TimedPose
{
  double stamp_sec{0.0};
  tf2::Transform odom_to_base;
};

struct LaserProjection
{
  double angle_min{-3.14159265358979323846};
  double angle_max{3.14159265358979323846};
  double angle_increment{0.01};
  double scan_time{0.1};
  double range_min{0.45};
  double range_max{12.0};
};

bool interpolate_pose(
  const std::vector<TimedPose> & poses,
  double query_sec,
  double max_gap_sec,
  tf2::Transform & result,
  std::string & error);

tf2::Vector3 deskew_point_to_reference(
  const tf2::Vector3 & point_in_sensor,
  const tf2::Transform & sensor_to_base,
  const tf2::Transform & odom_to_base_at_point,
  const tf2::Transform & odom_to_base_at_reference);

bool extract_absolute_nanosecond_timestamps(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & field_name,
  std::vector<double> & timestamps_sec,
  std::string & error);

sensor_msgs::msg::PointCloud2 select_points_preserving_fields(
  const sensor_msgs::msg::PointCloud2 & input,
  const std::vector<std::size_t> & kept_linear_indices);

sensor_msgs::msg::LaserScan make_laser_scan(
  const std_msgs::msg::Header & source_header,
  const std::string & target_frame,
  const LaserProjection & projection,
  std::vector<float> ranges);

}  // namespace rm_mid360_driver_bridge
