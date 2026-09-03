#include "rm_mid360_driver_bridge/livox_message_conversion.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "sensor_msgs/msg/point_field.hpp"

namespace rm_mid360_driver_bridge
{
namespace
{

constexpr std::uint32_t kPointStep = 26U;

template<typename T>
void write_value(std::uint8_t * destination, const T & value)
{
  std::memcpy(destination, &value, sizeof(T));
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

}  // namespace

bool custom_msg_to_pointcloud2(
  const livox_ros_driver2::msg::CustomMsg & input,
  const std::string & output_frame_id,
  sensor_msgs::msg::PointCloud2 & output,
  std::string & error)
{
  error.clear();
  output = sensor_msgs::msg::PointCloud2{};

  if (output_frame_id.empty()) {
    error = "output frame is empty";
    return false;
  }
  if (input.timebase == 0U ||
    (input.header.stamp.sec == 0 && input.header.stamp.nanosec == 0))
  {
    error = "native CustomMsg has a zero timebase or header stamp";
    return false;
  }
  if (input.point_num != input.points.size()) {
    error = "native CustomMsg point_num does not match points.size";
    return false;
  }
  if (input.points.size() > std::numeric_limits<std::uint32_t>::max()) {
    error = "native CustomMsg contains too many points";
    return false;
  }

  output.header = input.header;
  output.header.frame_id = output_frame_id;
  output.height = 1U;
  output.width = static_cast<std::uint32_t>(input.points.size());
  output.fields = {
    field("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
    field("intensity", 12U, sensor_msgs::msg::PointField::FLOAT32),
    field("tag", 16U, sensor_msgs::msg::PointField::UINT8),
    field("line", 17U, sensor_msgs::msg::PointField::UINT8),
    field("timestamp", 18U, sensor_msgs::msg::PointField::FLOAT64),
  };
  output.is_bigendian = false;
  output.point_step = kPointStep;
  output.row_step = output.point_step * output.width;
  output.is_dense = true;
  output.data.resize(output.row_step);

  for (std::size_t index = 0; index < input.points.size(); ++index) {
    const auto & source = input.points[index];
    if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
      !std::isfinite(source.z))
    {
      output.is_dense = false;
    }

    auto * destination = output.data.data() + index * output.point_step;
    const float intensity = static_cast<float>(source.reflectivity);
    const double absolute_time_ns = static_cast<double>(input.timebase) +
      static_cast<double>(source.offset_time);
    write_value(destination + 0U, source.x);
    write_value(destination + 4U, source.y);
    write_value(destination + 8U, source.z);
    write_value(destination + 12U, intensity);
    write_value(destination + 16U, source.tag);
    write_value(destination + 17U, source.line);
    write_value(destination + 18U, absolute_time_ns);
  }
  return true;
}

}  // namespace rm_mid360_driver_bridge
