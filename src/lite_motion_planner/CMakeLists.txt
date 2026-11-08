cmake_minimum_required(VERSION 3.16)
project(lite_motion_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arm_task_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lite_motion_msgs REQUIRED)
find_package(arm2nav_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(ompl CONFIG REQUIRED)

# Ubuntu 22.04's OMPL 1.5 exports variables rather than an imported target.
if(NOT TARGET ompl::ompl)
  add_library(ompl::ompl INTERFACE IMPORTED)
  set_target_properties(ompl::ompl PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OMPL_INCLUDE_DIRS}"
    INTERFACE_LINK_DIRECTORIES "${OMPL_LIBRARY_DIRS}"
    INTERFACE_LINK_LIBRARIES "${OMPL_LIBRARIES}"
  )
endif()

# tinyxml2: first try config package, then module package.
find_package(TinyXML2 QUIET)
if(NOT TinyXML2_FOUND)
  find_package(tinyxml2 REQUIRED)
endif()

find_package(ruckig REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/robot_model.cpp
  src/kinematics_solver.cpp
  src/collision_scene.cpp
  src/motion_planner.cpp
  src/trajectory_generator.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  pinocchio::pinocchio
  coal::coal
  ompl::ompl
)

if(TARGET tinyxml2::tinyxml2)
  target_link_libraries(${PROJECT_NAME}_core tinyxml2::tinyxml2)
elseif(TARGET tinyxml2)
  target_link_libraries(${PROJECT_NAME}_core tinyxml2)
endif()

if(TARGET ruckig::ruckig)
  target_link_libraries(${PROJECT_NAME}_core ruckig::ruckig)
elseif(TARGET ruckig)
  target_link_libraries(${PROJECT_NAME}_core ruckig)
endif()

ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  rclcpp
  trajectory_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  moveit_msgs
  lite_motion_msgs
  geometric_shapes
  pinocchio
)

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node ${PROJECT_NAME}_core)

set(${PROJECT_NAME}_executables planning_node)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/haomu_catch.cpp")
  add_executable(haomu_catch src/haomu_catch.cpp)
  target_link_libraries(haomu_catch ${PROJECT_NAME}_core)
  list(APPEND ${PROJECT_NAME}_executables haomu_catch)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/new_haomu_catch.cpp")
  add_executable(record_haomu_catch_fixed_tail src/new_haomu_catch.cpp)
  target_compile_definitions(record_haomu_catch_fixed_tail PRIVATE LITE_MOTION_FIXED_TAIL_RECORDER)
  target_link_libraries(record_haomu_catch_fixed_tail ${PROJECT_NAME}_core)
  list(APPEND ${PROJECT_NAME}_executables record_haomu_catch_fixed_tail)

  add_executable(record_dhz_4_fixed_head src/new_haomu_catch.cpp)
  target_compile_definitions(record_dhz_4_fixed_head PRIVATE LITE_MOTION_DHZ4_FIXED_HEAD_RECORDER)
  target_link_libraries(record_dhz_4_fixed_head ${PROJECT_NAME}_core)
  list(APPEND ${PROJECT_NAME}_executables record_dhz_4_fixed_head)

  add_executable(new_haomu_catch src/new_haomu_catch.cpp)
  target_compile_definitions(new_haomu_catch PRIVATE LITE_MOTION_FIXED_TAIL_REPLAYER)
  target_link_libraries(new_haomu_catch ${PROJECT_NAME}_core)
  list(APPEND ${PROJECT_NAME}_executables new_haomu_catch)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/dhz_4.cpp")
  add_executable(dhz_4 src/dhz_4.cpp)
  target_link_libraries(dhz_4 ${PROJECT_NAME}_core)
  list(APPEND ${PROJECT_NAME}_executables dhz_4)
endif()

add_executable(tzb_catch src/tzb_catch.cpp)
target_link_libraries(tzb_catch ${PROJECT_NAME}_core)
target_link_libraries(tzb_catch nlohmann_json::nlohmann_json)
list(APPEND ${PROJECT_NAME}_executables tzb_catch)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/haomu_flow_console.cpp")
  add_executable(haomu_flow_console src/haomu_flow_console.cpp)
  target_link_libraries(haomu_flow_console ${PROJECT_NAME}_core)
  list(APPEND ${PROJECT_NAME}_executables haomu_flow_console)
endif()

ament_target_dependencies(planning_node
  ament_index_cpp rclcpp trajectory_msgs geometry_msgs shape_msgs std_msgs sensor_msgs visualization_msgs moveit_msgs lite_motion_msgs geometric_shapes
)

if(TARGET haomu_catch)
  ament_target_dependencies(haomu_catch
    rclcpp trajectory_msgs geometry_msgs visualization_msgs lite_motion_msgs
  )
endif()

if(TARGET record_haomu_catch_fixed_tail)
  ament_target_dependencies(record_haomu_catch_fixed_tail
    rclcpp trajectory_msgs geometry_msgs sensor_msgs visualization_msgs lite_motion_msgs
  )
endif()

if(TARGET record_dhz_4_fixed_head)
  ament_target_dependencies(record_dhz_4_fixed_head
    rclcpp trajectory_msgs geometry_msgs sensor_msgs visualization_msgs lite_motion_msgs
  )
endif()

if(TARGET new_haomu_catch)
  ament_target_dependencies(new_haomu_catch
    rclcpp trajectory_msgs geometry_msgs sensor_msgs visualization_msgs lite_motion_msgs
  )
endif()

if(TARGET dhz_4)
  ament_target_dependencies(dhz_4
    rclcpp trajectory_msgs geometry_msgs visualization_msgs lite_motion_msgs
  )
endif()

ament_target_dependencies(tzb_catch
  rclcpp rclcpp_action trajectory_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs tf2_ros lite_motion_msgs arm2nav_msgs arm_task_interfaces
)

if(TARGET haomu_flow_console)
  ament_target_dependencies(haomu_flow_console
    rclcpp trajectory_msgs geometry_msgs visualization_msgs lite_motion_msgs
  )
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core ${${PROJECT_NAME}_executables}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_test(NAME test_mission_arm
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_control.py
      $<TARGET_FILE:tzb_catch> ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(test_mission_arm PROPERTIES TIMEOUT 150 RUN_SERIAL TRUE
    ENVIRONMENT "ROS_DOMAIN_ID=223;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/rover_arm_mission_tests")
endif()

ament_package()
