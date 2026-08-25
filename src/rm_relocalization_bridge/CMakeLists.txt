cmake_minimum_required(VERSION 3.8)
project(rm_relocalization_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(relocalization_math src/relocalization_math.cpp)
target_include_directories(relocalization_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(relocalization_math tf2)

add_executable(map_odom_from_global_pose src/map_odom_from_global_pose.cpp)
target_link_libraries(map_odom_from_global_pose relocalization_math)
ament_target_dependencies(map_odom_from_global_pose
  builtin_interfaces
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

add_executable(fake_global_pose_publisher src/fake_global_pose_publisher.cpp)
ament_target_dependencies(fake_global_pose_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

add_library(global_pose_gate src/global_pose_gate.cpp)
target_include_directories(global_pose_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(global_pose_gate geometry_msgs)

add_executable(global_pose_gate_node src/amcl_pose_gate_node.cpp)
target_link_libraries(global_pose_gate_node global_pose_gate)
ament_target_dependencies(global_pose_gate_node
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(fake_2d_localization_world src/fake_2d_localization_world.cpp)
ament_target_dependencies(fake_2d_localization_world
  rclcpp
  nav_msgs
  sensor_msgs
)

install(TARGETS
  relocalization_math
  map_odom_from_global_pose
  fake_global_pose_publisher
  global_pose_gate
  global_pose_gate_node
  fake_2d_localization_world
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
  ament_add_gtest(test_relocalization_math test/test_relocalization_math.cpp)
  target_link_libraries(test_relocalization_math relocalization_math)
  ament_target_dependencies(test_relocalization_math tf2)

  ament_add_gtest(test_global_pose_gate test/test_global_pose_gate.cpp)
  target_link_libraries(test_global_pose_gate global_pose_gate)
  ament_target_dependencies(test_global_pose_gate geometry_msgs)

  ament_add_pytest_test(
    test_spin_candidate_profiles
    test/test_spin_candidate_profiles.py
    APPEND_ENV "RM_RELOCALIZATION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
  )

  ament_add_pytest_test(
    test_map_odom_autonomous_recovery
    test/test_map_odom_autonomous_recovery.py
    TIMEOUT 30
    APPEND_ENV
      "RM_RELOCALIZATION_BRIDGE_BINARY=${CMAKE_CURRENT_BINARY_DIR}/map_odom_from_global_pose"
      "RM_RELOCALIZATION_BRIDGE_OLD_CAR_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/map_odom_from_global_pose_old_car_2026.yaml"
  )
endif()

ament_package()
