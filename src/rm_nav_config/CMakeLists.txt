cmake_minimum_required(VERSION 3.8)
project(rm_nav_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  behavior_trees
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/costmap_inspector
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_old_car_global_dynamic_replanning
    test/test_old_car_global_dynamic_replanning.py
    APPEND_ENV "RM_NAV_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
