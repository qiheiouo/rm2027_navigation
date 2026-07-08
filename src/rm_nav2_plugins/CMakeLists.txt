cmake_minimum_required(VERSION 3.8)
project(rm_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(timed_obstacle_layer SHARED
  src/timed_obstacle_layer.cpp
)
target_compile_features(timed_obstacle_layer PUBLIC cxx_std_17)
target_include_directories(timed_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(timed_obstacle_layer
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_costmap_2d timed_obstacle_layer.xml)

install(TARGETS timed_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(timed_obstacle_layer)
ament_export_dependencies(
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
