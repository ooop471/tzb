# CMake generated Testfile for 
# Source directory: /home/tzb/nav_explore_ws/src/navigation/terrain_analysis
# Build directory: /home/tzb/nav_explore_ws/build/terrain_analysis
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(terrain_realtime "/home/tzb/nav_explore_ws/build/terrain_analysis/test_terrain_realtime")
set_tests_properties(terrain_realtime PROPERTIES  _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/navigation/terrain_analysis/CMakeLists.txt;62;add_test;/home/tzb/nav_explore_ws/src/navigation/terrain_analysis/CMakeLists.txt;0;")
add_test(obstacle_retention "/usr/bin/python3" "/home/tzb/nav_explore_ws/src/navigation/terrain_analysis/tests/test_obstacle_retention.py" "/home/tzb/nav_explore_ws/build/terrain_analysis/terrainAnalysis")
set_tests_properties(obstacle_retention PROPERTIES  ENVIRONMENT "ROS_DOMAIN_ID=230;ROS_LOCALHOST_ONLY=1" TIMEOUT "40" _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/navigation/terrain_analysis/CMakeLists.txt;63;add_test;/home/tzb/nav_explore_ws/src/navigation/terrain_analysis/CMakeLists.txt;0;")
