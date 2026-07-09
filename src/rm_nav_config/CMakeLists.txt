cmake_minimum_required(VERSION 3.8)
project(rm_nav_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/costmap_inspector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
