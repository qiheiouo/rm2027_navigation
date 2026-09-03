#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rm_mid360_driver_bridge/livox_message_conversion.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

template<typename T>
T read_value(const sensor_msgs::msg::PointCloud2 & cloud, std::size_t offset)
{
  T value{};
  std::memcpy(&value, cloud.data.data() + offset, sizeof(T));
  return value;
}

livox_ros_driver2::msg::CustomMsg message()
{
  livox_ros_driver2::msg::CustomMsg result;
  result.header.stamp.sec = 100;
  result.header.stamp.nanosec = 1000U;
  result.header.frame_id = "livox_frame";
  result.timebase = 100000001000ULL;
  result.lidar_id = 7U;

  livox_ros_driver2::msg::CustomPoint first;
  first.x = 1.0F;
  first.y = 2.0F;
  first.z = 3.0F;
  first.reflectivity = 12U;
  first.tag = 0x10U;
  first.line = 1U;
  first.offset_time = 0U;
  result.points.push_back(first);

  auto second = first;
  second.x = -1.0F;
  second.reflectivity = 255U;
  second.tag = 0x20U;
  second.line = 2U;
  second.offset_time = 2000U;
  result.points.push_back(second);
  result.point_num = static_cast<std::uint32_t>(result.points.size());
  return result;
}

TEST(LivoxMessageConversion, PreservesNativeTimingAndFields)
{
  sensor_msgs::msg::PointCloud2 cloud;
  std::string error;
  ASSERT_TRUE(rm_mid360_driver_bridge::custom_msg_to_pointcloud2(
      message(), "mid360_left_frame", cloud, error)) << error;

  EXPECT_EQ(cloud.header.frame_id, "mid360_left_frame");
  EXPECT_EQ(cloud.header.stamp.sec, 100);
  EXPECT_EQ(cloud.header.stamp.nanosec, 1000U);
  EXPECT_EQ(cloud.width, 2U);
  EXPECT_EQ(cloud.point_step, 26U);
  EXPECT_EQ(cloud.row_step, 52U);
  ASSERT_EQ(cloud.fields.size(), 7U);
  EXPECT_EQ(cloud.fields[6].name, "timestamp");
  EXPECT_EQ(cloud.fields[6].offset, 18U);
  EXPECT_EQ(cloud.fields[6].datatype, sensor_msgs::msg::PointField::FLOAT64);

  EXPECT_FLOAT_EQ(read_value<float>(cloud, 0U), 1.0F);
  EXPECT_FLOAT_EQ(read_value<float>(cloud, 12U), 12.0F);
  EXPECT_EQ(read_value<std::uint8_t>(cloud, 16U), 0x10U);
  EXPECT_EQ(read_value<std::uint8_t>(cloud, 17U), 1U);
  EXPECT_DOUBLE_EQ(read_value<double>(cloud, 18U), 100000001000.0);

  const std::size_t second = cloud.point_step;
  EXPECT_FLOAT_EQ(read_value<float>(cloud, second), -1.0F);
  EXPECT_FLOAT_EQ(read_value<float>(cloud, second + 12U), 255.0F);
  EXPECT_DOUBLE_EQ(read_value<double>(cloud, second + 18U), 100000003000.0);
}

TEST(LivoxMessageConversion, RejectsPointCountMismatch)
{
  auto input = message();
  input.point_num = 3U;
  sensor_msgs::msg::PointCloud2 cloud;
  std::string error;
  EXPECT_FALSE(rm_mid360_driver_bridge::custom_msg_to_pointcloud2(
      input, "mid360_left_frame", cloud, error));
  EXPECT_NE(error.find("point_num"), std::string::npos);
}

TEST(LivoxMessageConversion, MarksNonFiniteCloudNonDense)
{
  auto input = message();
  input.points[0].x = std::numeric_limits<float>::quiet_NaN();
  sensor_msgs::msg::PointCloud2 cloud;
  std::string error;
  ASSERT_TRUE(rm_mid360_driver_bridge::custom_msg_to_pointcloud2(
      input, "mid360_left_frame", cloud, error)) << error;
  EXPECT_FALSE(cloud.is_dense);
}

}  // namespace
