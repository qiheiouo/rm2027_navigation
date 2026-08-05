cmake_minimum_required(VERSION 3.8)
project(rm_dog_hole)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(dog_hole_geometry src/geometry.cpp)
target_include_directories(dog_hole_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dog_hole_manager src/dog_hole_manager.cpp)
target_link_libraries(dog_hole_manager dog_hole_geometry)
ament_target_dependencies(dog_hole_manager
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

install(TARGETS
  dog_hole_geometry
  dog_hole_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dog_hole_capture_matrix.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dog_hole_capture_matrix
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dog_hole_geometry test/test_dog_hole_geometry.cpp)
  target_link_libraries(test_dog_hole_geometry dog_hole_geometry)
endif()

ament_package()
