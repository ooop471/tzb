# CMake generated Testfile for 
# Source directory: /home/tzb/nav_explore_ws/src/explore
# Build directory: /home/tzb/nav_explore_ws/build/rover_explore
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(corridor_dead_end "/home/tzb/nav_explore_ws/build/rover_explore/test_corridor_dead_end")
set_tests_properties(corridor_dead_end PROPERTIES  _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/explore/CMakeLists.txt;90;add_test;/home/tzb/nav_explore_ws/src/explore/CMakeLists.txt;0;")
add_test(ceiling_exit "/home/tzb/nav_explore_ws/build/rover_explore/test_ceiling_exit")
set_tests_properties(ceiling_exit PROPERTIES  _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/explore/CMakeLists.txt;94;add_test;/home/tzb/nav_explore_ws/src/explore/CMakeLists.txt;0;")
add_test(exploration_waiting "/home/tzb/nav_explore_ws/build/rover_explore/test_exploration_waiting")
set_tests_properties(exploration_waiting PROPERTIES  ENVIRONMENT "ROS_DOMAIN_ID=228;ROS_LOCALHOST_ONLY=1" _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/explore/CMakeLists.txt;98;add_test;/home/tzb/nav_explore_ws/src/explore/CMakeLists.txt;0;")
