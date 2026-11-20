cmake_minimum_required(VERSION 3.5)
project(terrain_analysis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(terrainAnalysis src/terrain_analysis_plus.cpp)
ament_target_dependencies(
  terrainAnalysis
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  pcl_ros
  pcl_conversions)

install(TARGETS terrainAnalysis 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch config 
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  enable_testing()
  add_executable(test_terrain_realtime tests/test_terrain_realtime.cpp)
  add_test(NAME terrain_realtime COMMAND test_terrain_realtime)
  add_test(NAME obstacle_retention COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_obstacle_retention.py $<TARGET_FILE:terrainAnalysis>)
  set_tests_properties(obstacle_retention PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=230;ROS_LOCALHOST_ONLY=1" TIMEOUT 40)
endif()

ament_package()
