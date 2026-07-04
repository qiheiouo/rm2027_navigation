cmake_minimum_required(VERSION 3.8)
project(rm_serial_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(rm_serial_protocol src/protocol.cpp)
target_include_directories(rm_serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(serial_dry_run_node src/serial_dry_run_node.cpp)
target_link_libraries(serial_dry_run_node rm_serial_protocol)
ament_target_dependencies(serial_dry_run_node
  geometry_msgs
  rclcpp
  std_msgs
)

add_executable(serial_transport_node src/serial_transport_node.cpp)
target_link_libraries(serial_transport_node rm_serial_protocol)
ament_target_dependencies(serial_transport_node
  geometry_msgs
  rclcpp
)

install(TARGETS rm_serial_protocol
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  serial_dry_run_node
  serial_transport_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol rm_serial_protocol)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(rm_serial_protocol)
ament_package()
