cmake_minimum_required(VERSION 3.16)
project(new_car_dual_probes LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
# Reuse the existing actual per-message GID observer without Gazebo dependencies.
add_executable(s2_tf_owner_probe ../../src/rm_new_car_simulation/src/s2_tf_owner_probe.cpp)
target_compile_features(s2_tf_owner_probe PRIVATE cxx_std_17)
ament_target_dependencies(s2_tf_owner_probe rclcpp tf2_msgs)
