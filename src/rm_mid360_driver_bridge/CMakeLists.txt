cmake_minimum_required(VERSION 3.8)
project(rm_mid360_driver_bridge)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pointcloud_self_filter_node src/pointcloud_self_filter_node.cpp)
target_compile_features(pointcloud_self_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_self_filter_node
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
target_compile_features(pointcloud_to_laserscan_node PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_to_laserscan_node
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  pointcloud_self_filter_node
  pointcloud_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
