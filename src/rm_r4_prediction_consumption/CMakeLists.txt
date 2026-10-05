cmake_minimum_required(VERSION 3.16)
project(rm_r4_prediction_consumption LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rm_r4_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)

# One unchanged provider at the existing migration path; no frontend clone.
set(TDT_SFC ${CMAKE_CURRENT_SOURCE_DIR}/../../experiments/tdt_planner/rm_tdt_planner/vendor/tdt_nav)
add_library(${PROJECT_NAME} SHARED src/consumption.cpp src/corridor.cpp
  ${TDT_SFC}/MinimumSnapOsqp/sfcSquare.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17 PRIVATE cxx_std_20)
target_compile_definitions(${PROJECT_NAME} PRIVATE RM_TDT_NO_OPENCV)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PRIVATE ${TDT_SFC}/MinimumSnapOsqp)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen OpenSSL::Crypto)
ament_target_dependencies(${PROJECT_NAME} PUBLIC nav_msgs rm_r4_interfaces)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
option(RM_R4_BUILD_FOLLOW "Build the optional fixed OSQP 0.6.3 Follow value solver" OFF)
if(RM_R4_BUILD_FOLLOW)
  # Installed by the existing fixed dependency intake, never fetched at configure.
  # 0.6.3 exports no CMake version file: follow.cpp checks header and runtime ABI.
  find_package(osqp REQUIRED CONFIG)
  find_package(rm_navigation_execution_adapters REQUIRED)
  add_library(${PROJECT_NAME}_follow SHARED src/follow.cpp)
  target_compile_features(${PROJECT_NAME}_follow PUBLIC cxx_std_17 PRIVATE cxx_std_20)
  target_link_libraries(${PROJECT_NAME}_follow PUBLIC ${PROJECT_NAME}
    PRIVATE Eigen3::Eigen OpenSSL::Crypto osqp::osqp
    rm_navigation_execution_adapters::rm_navigation_execution_adapters)
  target_compile_options(${PROJECT_NAME}_follow PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS ${PROJECT_NAME}_follow EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
  install(FILES include/${PROJECT_NAME}/follow.hpp DESTINATION include/${PROJECT_NAME})
  ament_export_dependencies(rm_navigation_execution_adapters)
endif()
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME canonical_cdr_producer COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/write_canonical_cdr.py ${CMAKE_CURRENT_BINARY_DIR}/producer_cdr)
  set_tests_properties(canonical_cdr_producer PROPERTIES TIMEOUT 15
    FIXTURES_SETUP r4_cdr ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  ament_add_gtest(test_consumption test/test_consumption.cpp TIMEOUT 30)
  target_link_libraries(test_consumption ${PROJECT_NAME})
  ament_target_dependencies(test_consumption rclcpp)
  set_tests_properties(test_consumption PROPERTIES FIXTURES_REQUIRED r4_cdr
    ENVIRONMENT "R4_PRODUCER_CDR_FIXTURES=${CMAKE_CURRENT_BINARY_DIR}/producer_cdr")
  add_executable(math_probe test/math_probe.cpp)
  target_link_libraries(math_probe ${PROJECT_NAME})
  add_test(NAME fixed_A02_soft_math COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_math_reference.py $<TARGET_FILE:math_probe>)
  set_tests_properties(fixed_A02_soft_math PROPERTIES TIMEOUT 15
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  ament_add_gtest(test_corridor test/test_corridor.cpp TIMEOUT 30)
  target_link_libraries(test_corridor ${PROJECT_NAME})
  if(RM_R4_BUILD_FOLLOW)
    ament_add_gtest(test_follow test/test_follow.cpp TIMEOUT 30)
    target_link_libraries(test_follow ${PROJECT_NAME}_follow)
    add_executable(follow_probe test/follow_probe.cpp)
    target_link_libraries(follow_probe ${PROJECT_NAME}_follow)
  endif()
endif()
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include PATTERN "follow.hpp" EXCLUDE)
install(FILES README.md ${TDT_SFC}/LICENCE ${TDT_SFC}/UPSTREAM.json
  ${TDT_SFC}/LOCAL_CHANGES.patch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(nav_msgs rm_r4_interfaces)
ament_package()
