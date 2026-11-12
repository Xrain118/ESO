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

add_executable(pregme_control_node src/pregme_control_node.cpp)
target_compile_options(pregme_control_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(pregme_control_node
  rclcpp px4_msgs eso_interfaces pregme_core)

install(TARGETS pregme_control_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(
    NAME source_contract
    COMMAND ${CMAKE_COMMAND}
      -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/pregme_control_node.cpp
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/source_contract.cmake)
endif()

ament_package()
