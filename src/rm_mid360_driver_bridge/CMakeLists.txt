cmake_minimum_required(VERSION 3.8)
project(rm_mid360_driver_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(pointcloud_deskew src/pointcloud_deskew.cpp)
target_include_directories(pointcloud_deskew PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud_deskew
  sensor_msgs
  std_msgs
  tf2
)

add_library(ramp_plane_filter src/ramp_plane_filter.cpp)
target_include_directories(ramp_plane_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pointcloud_self_filter_node src/pointcloud_self_filter_node.cpp)
target_compile_features(pointcloud_self_filter_node PUBLIC cxx_std_17)
target_link_libraries(pointcloud_self_filter_node pointcloud_deskew)
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
target_link_libraries(pointcloud_to_laserscan_node pointcloud_deskew)
ament_target_dependencies(pointcloud_to_laserscan_node
  diagnostic_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(pointcloud_fusion_node src/pointcloud_fusion_node.cpp)
target_compile_features(pointcloud_fusion_node PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(ramp_plane_filter_node src/ramp_plane_filter_node.cpp)
target_compile_features(ramp_plane_filter_node PUBLIC cxx_std_17)
target_link_libraries(ramp_plane_filter_node
  pointcloud_deskew
  ramp_plane_filter
)
ament_target_dependencies(ramp_plane_filter_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(ramp_laserscan_filter_node
  src/ramp_laserscan_filter_node.cpp
)
target_compile_features(ramp_laserscan_filter_node PUBLIC cxx_std_17)
target_link_libraries(ramp_laserscan_filter_node ramp_plane_filter)
ament_target_dependencies(ramp_laserscan_filter_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(livox_pointcloud_adapter_node src/livox_pointcloud_adapter_node.cpp)
target_compile_features(livox_pointcloud_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(livox_pointcloud_adapter_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

install(TARGETS
  pointcloud_deskew
  ramp_plane_filter
  pointcloud_self_filter_node
  pointcloud_to_laserscan_node
  pointcloud_fusion_node
  ramp_plane_filter_node
  ramp_laserscan_filter_node
  livox_pointcloud_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_pointcloud_deskew test/test_pointcloud_deskew.cpp)
  target_link_libraries(test_pointcloud_deskew pointcloud_deskew)
  ament_target_dependencies(test_pointcloud_deskew sensor_msgs std_msgs tf2)

  ament_add_gtest(test_ramp_plane_filter test/test_ramp_plane_filter.cpp)
  target_link_libraries(test_ramp_plane_filter ramp_plane_filter)

  ament_add_pytest_test(
    test_pointcloud_fusion_runtime
    test/test_pointcloud_fusion_runtime.py
    TIMEOUT 40
  )
  ament_add_pytest_test(
    test_livox_pointcloud_adapter_runtime
    test/test_livox_pointcloud_adapter_runtime.py
    TIMEOUT 30
  )
endif()

ament_package()
