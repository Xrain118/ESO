cmake_minimum_required(VERSION 3.8)
project(pregme_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eso_interfaces REQUIRED)
find_package(pregme_core REQUIRED)

add_library(pregme_runtime_config src/runtime_config.cpp)
target_include_directories(pregme_runtime_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(pregme_runtime_config PRIVATE -Wall -Wextra -Wpedantic)

add_executable(pregme_control_node src/pregme_control_node.cpp)
target_compile_options(pregme_control_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(pregme_control_node
  rclcpp px4_msgs eso_interfaces pregme_core)
target_link_libraries(pregme_control_node pregme_runtime_config)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS pregme_runtime_config pregme_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_runtime_config test/test_runtime_config.cpp)
  target_link_libraries(test_runtime_config pregme_runtime_config)
  add_launch_test(test/test_node_safety.py TIMEOUT 30)
  add_test(
    NAME source_contract
    COMMAND ${CMAKE_COMMAND}
      -DSOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/source_contract.cmake)
  add_test(
    NAME config_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/config_contract.py
      --repo-root ${CMAKE_CURRENT_SOURCE_DIR}/../..)
endif()

ament_package()
