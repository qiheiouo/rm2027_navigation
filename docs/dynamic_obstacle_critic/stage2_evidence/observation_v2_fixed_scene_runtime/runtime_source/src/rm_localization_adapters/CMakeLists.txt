cmake_minimum_required(VERSION 3.8)
project(rm_localization_adapters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(canonical_odometry src/canonical_odometry.cpp)
target_include_directories(canonical_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(canonical_odometry
  geometry_msgs
  tf2
)

add_executable(map_odom_stub src/map_odom_stub.cpp)
ament_target_dependencies(map_odom_stub
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(lio_adapter src/lio_adapter.cpp)
target_link_libraries(lio_adapter canonical_odometry)
ament_target_dependencies(lio_adapter
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(gimbal_state_adapter src/gimbal_state_adapter.cpp)
ament_target_dependencies(gimbal_state_adapter
  rclcpp
  sensor_msgs
  builtin_interfaces
)

add_executable(imu_frame_adapter src/imu_frame_adapter.cpp)
ament_target_dependencies(imu_frame_adapter
  rclcpp
  sensor_msgs
)

add_executable(fake_lio_odom_publisher src/fake_lio_odom_publisher.cpp)
ament_target_dependencies(fake_lio_odom_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

install(TARGETS
  canonical_odometry
  map_odom_stub
  lio_adapter
  gimbal_state_adapter
  imu_frame_adapter
  fake_lio_odom_publisher
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
  ament_add_gtest(test_canonical_odometry test/test_canonical_odometry.cpp)
  target_link_libraries(test_canonical_odometry canonical_odometry)
  ament_target_dependencies(test_canonical_odometry geometry_msgs tf2)
endif()

ament_package()
