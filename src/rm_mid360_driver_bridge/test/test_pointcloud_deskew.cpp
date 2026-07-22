#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

#include "rm_mid360_driver_bridge/pointcloud_deskew.hpp"

namespace
{

tf2::Transform pose(double x, double y, double z, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  tf2::Transform result;
  result.setOrigin(tf2::Vector3(x, y, z));
  result.setRotation(q);
  return result;
}

sensor_msgs::msg::PointField field(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1U;
  return result;
}

template<typename T>
void write_value(std::vector<std::uint8_t> & data, std::size_t offset, T value)
{
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

sensor_msgs::msg::PointCloud2 livox_cloud(std::uint32_t width = 2U)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "mid360_left_frame";
  cloud.header.stamp.sec = 1784534094;
  cloud.header.stamp.nanosec = 158264900U;
  cloud.height = 1U;
  cloud.width = width;
  cloud.fields = {
    field("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
    field("intensity", 12U, sensor_msgs::msg::PointField::FLOAT32),
    field("tag", 16U, sensor_msgs::msg::PointField::UINT8),
    field("line", 17U, sensor_msgs::msg::PointField::UINT8),
    field("timestamp", 18U, sensor_msgs::msg::PointField::FLOAT64),
  };
  cloud.is_bigendian = false;
  cloud.point_step = 26U;
  cloud.row_step = width * cloud.point_step;
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step);
  for (std::uint32_t index = 0; index < width; ++index) {
    const std::size_t base = index * cloud.point_step;
    write_value<float>(cloud.data, base + 0U, static_cast<float>(index + 1U));
    write_value<float>(cloud.data, base + 4U, static_cast<float>(index + 2U));
    write_value<float>(cloud.data, base + 8U, static_cast<float>(index + 3U));
    write_value<float>(cloud.data, base + 12U, 42.0F + index);
    cloud.data[base + 16U] = static_cast<std::uint8_t>(7U + index);
    cloud.data[base + 17U] = static_cast<std::uint8_t>(2U + index);
    write_value<double>(
      cloud.data, base + 18U,
      1784534094158264900.0 + static_cast<double>(index) * 20000000.0);
  }
  return cloud;
}

double yaw_of(const tf2::Transform & transform)
{
  const auto q = transform.getRotation();
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}  // namespace

TEST(PointcloudDeskew, RejectsEmptyAndInvalidInterpolationQueries)
{
  tf2::Transform result;
  std::string error;
  EXPECT_FALSE(rm_mid360_driver_bridge::interpolate_pose({}, 1.0, 0.1, result, error));
  const std::vector<rm_mid360_driver_bridge::TimedPose> poses{
    {1.0, pose(0.0, 0.0, 0.0, 0.0)},
    {1.1, pose(1.0, 0.0, 0.0, 0.0)},
  };
  EXPECT_FALSE(
    rm_mid360_driver_bridge::interpolate_pose(
      poses, std::numeric_limits<double>::quiet_NaN(), 0.2, result, error));
  EXPECT_FALSE(
    rm_mid360_driver_bridge::interpolate_pose(
      poses, 0.9, 0.2, result, error));
  EXPECT_FALSE(
    rm_mid360_driver_bridge::interpolate_pose(
      poses, 1.05, 0.05, result, error));
}

TEST(PointcloudDeskew, InterpolatesTranslationAndZeroMotion)
{
  const std::vector<rm_mid360_driver_bridge::TimedPose> poses{
    {1.0, pose(0.0, -2.0, 3.0, 0.2)},
    {1.2, pose(2.0, 0.0, 5.0, 0.2)},
  };
  tf2::Transform result;
  std::string error;
  ASSERT_TRUE(
    rm_mid360_driver_bridge::interpolate_pose(
      poses, 1.1, 0.3, result, error)) << error;
  EXPECT_NEAR(result.getOrigin().x(), 1.0, 1.0e-9);
  EXPECT_NEAR(result.getOrigin().y(), -1.0, 1.0e-9);
  EXPECT_NEAR(result.getOrigin().z(), 4.0, 1.0e-9);
  EXPECT_NEAR(yaw_of(result), 0.2, 1.0e-9);
}

TEST(PointcloudDeskew, SlerpTakesShortPathAcrossPi)
{
  constexpr double kPi = 3.14159265358979323846;
  const std::vector<rm_mid360_driver_bridge::TimedPose> poses{
    {1.0, pose(0.0, 0.0, 0.0, 179.0 * kPi / 180.0)},
    {1.2, pose(0.0, 0.0, 0.0, -179.0 * kPi / 180.0)},
  };
  tf2::Transform result;
  std::string error;
  ASSERT_TRUE(
    rm_mid360_driver_bridge::interpolate_pose(
      poses, 1.1, 0.3, result, error)) << error;
  EXPECT_NEAR(std::abs(yaw_of(result)), kPi, 1.0e-6);
}

TEST(PointcloudDeskew, CompensatesYawTranslationAndStaticExtrinsic)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  const tf2::Vector3 point_sensor(1.0, 0.0, 0.0);
  const tf2::Transform sensor_to_base = pose(0.5, 0.0, 0.0, 0.0);
  const tf2::Transform point_pose = pose(1.0, 0.0, 0.0, kHalfPi);
  const tf2::Transform reference_pose = pose(0.0, 0.0, 0.0, 0.0);
  const tf2::Vector3 compensated =
    rm_mid360_driver_bridge::deskew_point_to_reference(
    point_sensor, sensor_to_base, point_pose, reference_pose);
  EXPECT_NEAR(compensated.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(compensated.y(), 1.5, 1.0e-9);
  EXPECT_NEAR(compensated.z(), 0.0, 1.0e-9);

  const tf2::Vector3 zero_motion =
    rm_mid360_driver_bridge::deskew_point_to_reference(
    point_sensor, sensor_to_base, reference_pose, reference_pose);
  EXPECT_NEAR(zero_motion.x(), 1.5, 1.0e-9);
  EXPECT_NEAR(zero_motion.y(), 0.0, 1.0e-9);
}

TEST(PointcloudDeskew, ReadsVerifiedLivoxAbsoluteNanoseconds)
{
  const auto cloud = livox_cloud();
  std::vector<double> timestamps;
  std::string error;
  ASSERT_TRUE(
    rm_mid360_driver_bridge::extract_absolute_nanosecond_timestamps(
      cloud, "timestamp", timestamps, error)) << error;
  ASSERT_EQ(timestamps.size(), 2U);
  EXPECT_NEAR(timestamps[0], 1784534094.1582649, 1.0e-6);
  EXPECT_NEAR(timestamps[1] - timestamps[0], 0.020, 1.0e-6);
}

TEST(PointcloudDeskew, RejectsMissingNaNAndEmptyTimestampInput)
{
  std::vector<double> timestamps;
  std::string error;
  auto cloud = livox_cloud(1U);
  EXPECT_FALSE(
    rm_mid360_driver_bridge::extract_absolute_nanosecond_timestamps(
      cloud, "not_present", timestamps, error));

  write_value<double>(
    cloud.data, 18U, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(
    rm_mid360_driver_bridge::extract_absolute_nanosecond_timestamps(
      cloud, "timestamp", timestamps, error));

  cloud.width = 0U;
  cloud.row_step = 0U;
  cloud.data.clear();
  EXPECT_FALSE(
    rm_mid360_driver_bridge::extract_absolute_nanosecond_timestamps(
      cloud, "timestamp", timestamps, error));
}

TEST(PointcloudDeskew, SelfFilterSelectionPreservesEveryFieldAndRecordByte)
{
  const auto input = livox_cloud();
  const auto output = rm_mid360_driver_bridge::select_points_preserving_fields(input, {1U});
  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.width, 1U);
  EXPECT_EQ(output.fields, input.fields);
  EXPECT_EQ(output.point_step, input.point_step);
  EXPECT_EQ(output.row_step, input.point_step);
  EXPECT_EQ(output.is_bigendian, input.is_bigendian);
  ASSERT_EQ(output.data.size(), input.point_step);
  EXPECT_TRUE(
    std::equal(
      output.data.begin(), output.data.end(), input.data.begin() + input.point_step));

  std::vector<double> timestamps;
  std::string error;
  ASSERT_TRUE(
    rm_mid360_driver_bridge::extract_absolute_nanosecond_timestamps(
      output, "timestamp", timestamps, error)) << error;
  ASSERT_EQ(timestamps.size(), 1U);
  EXPECT_NEAR(timestamps[0], 1784534094.1782649, 1.0e-6);
}

TEST(PointcloudDeskew, SelectionHandlesOrganizedCloudRowPadding)
{
  auto input = livox_cloud(2U);
  const std::vector<std::uint8_t> first_row = input.data;
  input.height = 2U;
  input.row_step = input.width * input.point_step + 4U;
  input.data.assign(static_cast<std::size_t>(input.row_step) * input.height, 0xEEU);
  std::copy(first_row.begin(), first_row.end(), input.data.begin());
  std::copy(
    first_row.begin(), first_row.end(),
    input.data.begin() + static_cast<std::ptrdiff_t>(input.row_step));
  input.data[input.row_step] = 0xABU;

  const auto output = rm_mid360_driver_bridge::select_points_preserving_fields(
    input, {1U, 2U});
  ASSERT_EQ(output.data.size(), 2U * input.point_step);
  EXPECT_TRUE(
    std::equal(
      output.data.begin(), output.data.begin() + input.point_step,
      input.data.begin() + input.point_step));
  EXPECT_EQ(output.data[input.point_step], 0xABU);
}

TEST(PointcloudDeskew, LaserScanPreservesSourceStampAndUsesTargetFrame)
{
  std_msgs::msg::Header header;
  header.stamp.sec = 123;
  header.stamp.nanosec = 456U;
  header.frame_id = "mid360_left_frame";
  rm_mid360_driver_bridge::LaserProjection projection;
  projection.scan_time = 0.1;
  const auto scan = rm_mid360_driver_bridge::make_laser_scan(
    header, "base_link", projection, {1.0F, 2.0F});
  EXPECT_EQ(scan.header.stamp, header.stamp);
  EXPECT_EQ(scan.header.frame_id, "base_link");
  EXPECT_FLOAT_EQ(scan.scan_time, 0.1F);
  ASSERT_EQ(scan.ranges.size(), 2U);
  EXPECT_FLOAT_EQ(scan.ranges[1], 2.0F);
}
