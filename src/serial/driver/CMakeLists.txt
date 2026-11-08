cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3 -Wall -Wpedantic -Werror)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_serial_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/packet.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  serial_driver
  io_context
  geometry_msgs
  sensor_msgs
  std_msgs
  rm_serial_msgs
)

# The component stays registered for composition; the standalone `node`
# executable uses an explicit main (src/node_main.cpp) so that SIGINT/SIGTERM
# honour the tzb_catch shutdown lease instead of rclcpp's default handler.
rclcpp_components_register_nodes(${PROJECT_NAME} "rm::serial::DriverNode")

add_executable(node src/node_main.cpp)
target_link_libraries(node ${PROJECT_NAME})
ament_target_dependencies(node rclcpp std_msgs)
install(TARGETS node DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  serial_driver
  io_context
  geometry_msgs
  sensor_msgs
  std_msgs
  rm_serial_msgs
)

ament_package()
