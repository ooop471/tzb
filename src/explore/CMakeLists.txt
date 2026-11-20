cmake_minimum_required(VERSION 3.16)
project(rover_explore)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(rover_explore_launch)
foreach(dep livox_ros_driver2 rclcpp rclcpp_action std_msgs std_srvs nav_msgs nav2_msgs sensor_msgs geometry_msgs
    visualization_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)
  find_package(${dep} REQUIRED)
endforeach()
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree segmentation)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(nlohmann_json REQUIRED)

# Keep platform-specific binaries outside the source package. Override on ARM.
set(ORTOOLS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/ortools" CACHE PATH
    "OR-Tools C++ installation containing include/ and lib/")
find_library(ORTOOLS_LIBRARY NAMES ortools PATHS "${ORTOOLS_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT ORTOOLS_LIBRARY OR NOT EXISTS "${ORTOOLS_ROOT}/include/ortools/constraint_solver/routing.h")
  message(FATAL_ERROR "Set ORTOOLS_ROOT to a matching OR-Tools C++ installation containing include/ and lib/")
endif()

set(core_sources
  src/rover_configuration.cpp
  src/lidar_model/lidar_model.cpp src/graph/graph.cpp
  src/utils/misc_utils.cpp src/utils/pointcloud_utils.cpp
  src/tsp_solver/tsp_solver.cpp src/viewpoint/viewpoint.cpp
  src/rolling_grid/rolling_grid.cpp src/viewpoint_manager/viewpoint_manager.cpp
  src/local_coverage_planner/local_coverage_planner.cpp src/grid_world/grid_world.cpp
  src/pointcloud_manager/pointcloud_manager.cpp src/keypose_graph/keypose_graph.cpp
  src/rolling_occupancy_grid/rolling_occupancy_grid.cpp src/planning_env/planning_env.cpp
  src/exploration_path/exploration_path.cpp src/tare_visualizer/tare_visualizer.cpp
  src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
add_library(rover_tare_core STATIC ${core_sources})
target_include_directories(rover_tare_core PUBLIC include ${PCL_INCLUDE_DIRS})
target_include_directories(rover_tare_core SYSTEM PUBLIC "${ORTOOLS_ROOT}/include")
ament_target_dependencies(rover_tare_core livox_ros_driver2 rclcpp std_msgs std_srvs nav_msgs nav2_msgs sensor_msgs
  geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)
target_link_libraries(rover_tare_core Eigen3::Eigen glog::glog nlohmann_json::nlohmann_json ${PCL_LIBRARIES} "${ORTOOLS_LIBRARY}")
add_executable(rover_explore_node src/tare_planner_node/tare_planner_node.cpp)
target_link_libraries(rover_explore_node rover_tare_core)
set_target_properties(rover_explore_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
install(TARGETS rover_explore_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY "${ORTOOLS_ROOT}/lib/" DESTINATION lib FILES_MATCHING PATTERN "libortools.so*")
add_library(exploration_follower_core src/exploration_path_follower.cpp)
target_compile_features(exploration_follower_core PUBLIC cxx_std_17)
target_include_directories(exploration_follower_core PUBLIC include)
ament_target_dependencies(exploration_follower_core
  rclcpp rclcpp_action nav2_msgs std_msgs nav_msgs sensor_msgs geometry_msgs
  tf2_ros tf2_geometry_msgs)
target_compile_options(exploration_follower_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(exploration_follower_core nlohmann_json::nlohmann_json)
add_executable(exploration_path_follower src/exploration_path_follower_main.cpp)
target_link_libraries(exploration_path_follower exploration_follower_core)

add_library(exploration_planner_core src/exploration_nav2_planner.cpp)
target_compile_features(exploration_planner_core PUBLIC cxx_std_17)
target_include_directories(exploration_planner_core PUBLIC include)
ament_target_dependencies(exploration_planner_core
  rclcpp rclcpp_action nav2_msgs std_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
target_compile_options(exploration_planner_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(exploration_nav2_planner src/exploration_nav2_planner_main.cpp)
target_link_libraries(exploration_nav2_planner exploration_planner_core)

add_library(rover_corridor_critic SHARED src/corridor_critic.cpp)
target_include_directories(rover_corridor_critic PRIVATE include)
ament_target_dependencies(rover_corridor_critic nav2_mppi_controller pluginlib nav2_costmap_2d
  geometry_msgs std_msgs tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_mppi_controller corridor_critic.xml)
install(TARGETS rover_corridor_critic LIBRARY DESTINATION lib)

install(TARGETS exploration_path_follower exploration_nav2_planner
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_corridor_dead_end tests/test_corridor_dead_end.cpp)
  target_include_directories(test_corridor_dead_end PRIVATE include)
  target_link_libraries(test_corridor_dead_end Eigen3::Eigen)
  target_compile_options(test_corridor_dead_end PRIVATE -UNDEBUG)
  add_test(NAME corridor_dead_end COMMAND test_corridor_dead_end)
  add_executable(test_ceiling_exit tests/test_ceiling_exit.cpp)
  target_include_directories(test_ceiling_exit PRIVATE include)
  target_link_libraries(test_ceiling_exit Eigen3::Eigen)
  add_test(NAME ceiling_exit COMMAND test_ceiling_exit)
  add_executable(test_exploration_waiting tests/test_exploration_waiting.cpp)
  target_link_libraries(test_exploration_waiting exploration_follower_core)
  target_compile_options(test_exploration_waiting PRIVATE -UNDEBUG)
  add_test(NAME exploration_waiting COMMAND test_exploration_waiting)
  set_tests_properties(exploration_waiting PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=228;ROS_LOCALHOST_ONLY=1")
endif()

ament_package()
