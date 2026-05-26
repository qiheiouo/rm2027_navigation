cmake_minimum_required(VERSION 3.8)
project(rm_chassis_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(chassis_interface_stub src/chassis_interface_stub.cpp)
ament_target_dependencies(chassis_interface_stub
  rclcpp
  geometry_msgs
  diagnostic_msgs
)

install(TARGETS
  chassis_interface_stub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
