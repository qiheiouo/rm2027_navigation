cmake_minimum_required(VERSION 3.8)
project(rm_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(scan_frame_adapter src/scan_frame_adapter.cpp)
ament_target_dependencies(scan_frame_adapter
  rclcpp
  sensor_msgs
)

add_executable(fake_pointcloud_obstacle_publisher
  src/fake_pointcloud_obstacle_publisher.cpp
)
ament_target_dependencies(fake_pointcloud_obstacle_publisher
  rclcpp
  sensor_msgs
)

add_executable(moving_obstacle_controller src/moving_obstacle_controller.cpp)
ament_target_dependencies(moving_obstacle_controller
  rclcpp
  std_msgs
)

add_executable(sim_gimbal_state_publisher src/sim_gimbal_state_publisher.cpp)
ament_target_dependencies(sim_gimbal_state_publisher
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  fake_pointcloud_obstacle_publisher
  scan_frame_adapter
  moving_obstacle_controller
  sim_gimbal_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
