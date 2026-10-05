cmake_minimum_required(VERSION 3.8)
project(rm_r4_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_competition_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObservedTrackMembers.msg"
  "msg/ObservedPredictionEnvelope.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs rm_competition_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
