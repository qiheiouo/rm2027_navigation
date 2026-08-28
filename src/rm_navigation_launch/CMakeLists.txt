cmake_minimum_required(VERSION 3.8)
project(rm_navigation_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
