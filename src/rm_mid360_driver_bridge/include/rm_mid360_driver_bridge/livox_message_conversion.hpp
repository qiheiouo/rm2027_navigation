#pragma once

#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace rm_mid360_driver_bridge
{

// Convert a native Livox CustomMsg into the PointCloud2 layout emitted by the
// pinned driver. The CustomMsg remains the timing authority for FAST-LIO; this
// conversion exists only for perception and visualization consumers.
bool custom_msg_to_pointcloud2(
  const livox_ros_driver2::msg::CustomMsg & input,
  const std::string & output_frame_id,
  sensor_msgs::msg::PointCloud2 & output,
  std::string & error);

}  // namespace rm_mid360_driver_bridge
