#include "rm_mid360_driver_bridge/pointcloud_deskew.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <sensor_msgs/msg/point_field.hpp>

namespace rm_mid360_driver_bridge
{
namespace
{

constexpr double kQuaternionNormEpsilon = 1.0e-12;

bool finite_transform(const tf2::Transform & transform)
{
  const auto & p = transform.getOrigin();
  const auto & q = transform.getRotation();
  return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) &&
         std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
         std::isfinite(q.w()) && q.length2() > kQuaternionNormEpsilon;
}

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & field_name)
{
  const auto found = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {
      return field.name == field_name;
    });
  return found == cloud.fields.end() ? nullptr : &(*found);
}

std::uint64_t byte_swap_u64(std::uint64_t value)
{
  return ((value & 0x00000000000000FFULL) << 56U) |
         ((value & 0x000000000000FF00ULL) << 40U) |
         ((value & 0x0000000000FF0000ULL) << 24U) |
         ((value & 0x00000000FF000000ULL) << 8U) |
         ((value & 0x000000FF00000000ULL) >> 8U) |
         ((value & 0x0000FF0000000000ULL) >> 24U) |
         ((value & 0x00FF000000000000ULL) >> 40U) |
         ((value & 0xFF00000000000000ULL) >> 56U);
}

double read_float64(const std::uint8_t * bytes, bool big_endian)
{
  std::uint64_t raw = 0;
  std::memcpy(&raw, bytes, sizeof(raw));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (big_endian) {
    raw = byte_swap_u64(raw);
  }
#else
  if (!big_endian) {
    raw = byte_swap_u64(raw);
  }
#endif
  double value = 0.0;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

}  // namespace

bool interpolate_pose(
  const std::vector<TimedPose> & poses,
  double query_sec,
  double max_gap_sec,
  tf2::Transform & result,
  std::string & error)
{
  if (poses.empty()) {
    error = "odometry buffer is empty";
    return false;
  }
  if (!std::isfinite(query_sec) || !std::isfinite(max_gap_sec) || max_gap_sec <= 0.0) {
    error = "query time or interpolation gap is invalid";
    return false;
  }
  if (query_sec < poses.front().stamp_sec || query_sec > poses.back().stamp_sec) {
    error = "query time is outside the odometry buffer";
    return false;
  }

  const auto upper = std::lower_bound(
    poses.begin(), poses.end(), query_sec,
    [](const TimedPose & pose, double stamp) {return pose.stamp_sec < stamp;});
  if (upper != poses.end() && std::abs(upper->stamp_sec - query_sec) <= 1.0e-9) {
    if (!finite_transform(upper->odom_to_base)) {
      error = "odometry pose is not finite";
      return false;
    }
    result = upper->odom_to_base;
    result.getRotation().normalize();
    return true;
  }
  if (upper == poses.begin() || upper == poses.end()) {
    error = "query time cannot be bracketed by odometry";
    return false;
  }

  const TimedPose & high = *upper;
  const TimedPose & low = *(upper - 1);
  const double gap = high.stamp_sec - low.stamp_sec;
  if (!std::isfinite(gap) || gap <= 0.0 || gap > max_gap_sec) {
    error = "odometry interpolation gap exceeds the configured limit";
    return false;
  }
  if (!finite_transform(low.odom_to_base) || !finite_transform(high.odom_to_base)) {
    error = "odometry pose is not finite";
    return false;
  }

  const double fraction = std::clamp((query_sec - low.stamp_sec) / gap, 0.0, 1.0);
  const tf2::Vector3 translation = low.odom_to_base.getOrigin().lerp(
    high.odom_to_base.getOrigin(), fraction);
  tf2::Quaternion q0 = low.odom_to_base.getRotation().normalized();
  tf2::Quaternion q1 = high.odom_to_base.getRotation().normalized();
  if (q0.dot(q1) < 0.0) {
    q1 = tf2::Quaternion(-q1.x(), -q1.y(), -q1.z(), -q1.w());
  }
  tf2::Quaternion rotation = q0.slerp(q1, fraction);
  rotation.normalize();
  result.setOrigin(translation);
  result.setRotation(rotation);
  return true;
}

tf2::Vector3 deskew_point_to_reference(
  const tf2::Vector3 & point_in_sensor,
  const tf2::Transform & sensor_to_base,
  const tf2::Transform & odom_to_base_at_point,
  const tf2::Transform & odom_to_base_at_reference)
{
  const tf2::Vector3 point_in_base_at_acquisition = sensor_to_base * point_in_sensor;
  const tf2::Vector3 point_in_odom =
    odom_to_base_at_point * point_in_base_at_acquisition;
  return odom_to_base_at_reference.inverse() * point_in_odom;
}

bool extract_absolute_nanosecond_timestamps(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & field_name,
  std::vector<double> & timestamps_sec,
  std::string & error)
{
  timestamps_sec.clear();
  const auto * field = find_field(cloud, field_name);
  if (field == nullptr) {
    error = "point timestamp field '" + field_name + "' is missing";
    return false;
  }
  if (field->datatype != sensor_msgs::msg::PointField::FLOAT64 || field->count != 1U) {
    error = "point timestamp field must be FLOAT64 with count=1";
    return false;
  }
  if (cloud.point_step == 0U || field->offset + sizeof(double) > cloud.point_step) {
    error = "point timestamp field lies outside point_step";
    return false;
  }
  if (cloud.width == 0U || cloud.height == 0U) {
    error = "pointcloud is empty";
    return false;
  }
  if (cloud.row_step < cloud.width * cloud.point_step ||
    cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height)
  {
    error = "pointcloud storage is truncated";
    return false;
  }

  timestamps_sec.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t column = 0; column < cloud.width; ++column) {
      const std::size_t offset = static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(column) * cloud.point_step + field->offset;
      const double absolute_nanoseconds = read_float64(
        cloud.data.data() + offset, cloud.is_bigendian);
      if (!std::isfinite(absolute_nanoseconds)) {
        error = "point timestamp contains NaN or infinity";
        timestamps_sec.clear();
        return false;
      }
      // Livox ROS driver2 writes PointXyzlt::offset_time here. Despite the
      // historical member name it is an absolute device/ROS timestamp in ns.
      timestamps_sec.push_back(absolute_nanoseconds * 1.0e-9);
    }
  }
  return true;
}

sensor_msgs::msg::PointCloud2 select_points_preserving_fields(
  const sensor_msgs::msg::PointCloud2 & input,
  const std::vector<std::size_t> & kept_linear_indices)
{
  if (input.point_step == 0U || input.row_step < input.width * input.point_step ||
    input.data.size() < static_cast<std::size_t>(input.row_step) * input.height)
  {
    throw std::invalid_argument("cannot select records from malformed PointCloud2 storage");
  }

  const std::size_t input_point_count =
    static_cast<std::size_t>(input.width) * input.height;
  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  output.height = 1U;
  output.width = static_cast<std::uint32_t>(kept_linear_indices.size());
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = output.width * output.point_step;
  output.is_dense = input.is_dense;
  output.data.reserve(static_cast<std::size_t>(output.row_step));

  for (const std::size_t index : kept_linear_indices) {
    if (index >= input_point_count) {
      throw std::out_of_range("selected PointCloud2 index is out of range");
    }
    const std::size_t row = index / input.width;
    const std::size_t column = index % input.width;
    const std::size_t offset = row * input.row_step + column * input.point_step;
    output.data.insert(
      output.data.end(), input.data.begin() + static_cast<std::ptrdiff_t>(offset),
      input.data.begin() + static_cast<std::ptrdiff_t>(offset + input.point_step));
  }
  return output;
}

sensor_msgs::msg::LaserScan make_laser_scan(
  const std_msgs::msg::Header & source_header,
  const std::string & target_frame,
  const LaserProjection & projection,
  std::vector<float> ranges)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header = source_header;
  if (!target_frame.empty()) {
    scan.header.frame_id = target_frame;
  }
  scan.angle_min = static_cast<float>(projection.angle_min);
  scan.angle_max = static_cast<float>(projection.angle_max);
  scan.angle_increment = static_cast<float>(projection.angle_increment);
  scan.time_increment = 0.0F;
  scan.scan_time = static_cast<float>(projection.scan_time);
  scan.range_min = static_cast<float>(projection.range_min);
  scan.range_max = static_cast<float>(projection.range_max);
  scan.ranges = std::move(ranges);
  return scan;
}

}  // namespace rm_mid360_driver_bridge
