cmake_minimum_required(VERSION 3.8)
project(rm_gicp_relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
# Compatibility shim for PCL/Qhull target naming on Ubuntu 22.04.
# PCL may reference QHULL::QHULL, while libqhull-dev exports Qhull::*.
find_package(Qhull CONFIG QUIET)

if(NOT TARGET QHULL::QHULL)
  if(TARGET Qhull::qhull_r)
    add_library(QHULL::QHULL INTERFACE IMPORTED GLOBAL)
    target_link_libraries(QHULL::QHULL INTERFACE Qhull::qhull_r)
  elseif(TARGET Qhull::libqhull)
    add_library(QHULL::QHULL INTERFACE IMPORTED GLOBAL)
    target_link_libraries(QHULL::QHULL INTERFACE Qhull::libqhull)
  else()
    find_library(QHULL_LIBRARY NAMES qhull_r qhull)
    if(QHULL_LIBRARY)
      add_library(QHULL::QHULL UNKNOWN IMPORTED GLOBAL)
      set_target_properties(QHULL::QHULL PROPERTIES
        IMPORTED_LOCATION "${QHULL_LIBRARY}"
      )
    endif()
  endif()
endif()

if(NOT TARGET QHULL::QHULL)
  message(FATAL_ERROR
    "PCL references QHULL::QHULL, but no compatible Qhull target or library was found. "
    "Install libqhull-dev."
  )
endif()
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(gicp_relocalization_math src/relocalization_math.cpp)
target_include_directories(gicp_relocalization_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gicp_relocalization_math Eigen3::Eigen)

add_executable(gicp_relocalization_node src/gicp_relocalization_node.cpp)
target_include_directories(gicp_relocalization_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(
  gicp_relocalization_node
  gicp_relocalization_math
  ${PCL_LIBRARIES}
)
ament_target_dependencies(gicp_relocalization_node
  builtin_interfaces
  geometry_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  tf2_ros
)

add_executable(fake_gicp_input src/fake_gicp_input.cpp)
target_include_directories(fake_gicp_input PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(fake_gicp_input ${PCL_LIBRARIES})
ament_target_dependencies(fake_gicp_input
  geometry_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
)

install(TARGETS
  gicp_relocalization_math
  gicp_relocalization_node
  fake_gicp_input
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_relocalization_math test/test_relocalization_math.cpp)
  target_link_libraries(test_relocalization_math gicp_relocalization_math Eigen3::Eigen)
endif()

ament_package()
