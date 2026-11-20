# CMake generated Testfile for 
# Source directory: /home/tzb/nav_explore_ws/src/state
# Build directory: /home/tzb/nav_explore_ws/build/rover_state
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cave_turnaround "/home/tzb/nav_explore_ws/build/rover_state/test_cave_turnaround")
set_tests_properties(cave_turnaround PROPERTIES  ENVIRONMENT "ROS_DOMAIN_ID=226;ROS_LOCALHOST_ONLY=1" _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;34;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
add_test(mission_budget "/home/tzb/nav_explore_ws/build/rover_state/test_mission_budget")
set_tests_properties(mission_budget PROPERTIES  _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;39;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
add_test(mission_cutoff "/home/tzb/nav_explore_ws/build/rover_state/test_mission_cutoff")
set_tests_properties(mission_cutoff PROPERTIES  ENVIRONMENT "ROS_DOMAIN_ID=227;ROS_LOCALHOST_ONLY=1" _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;43;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
add_test(outdoor_navigation "/home/tzb/nav_explore_ws/build/rover_state/test_outdoor_navigation")
set_tests_properties(outdoor_navigation PROPERTIES  ENVIRONMENT "ROS_DOMAIN_ID=229;ROS_LOCALHOST_ONLY=1" _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;47;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
add_test(outdoor_profile "/usr/bin/python3" "/home/tzb/nav_explore_ws/src/state/tests/test_outdoor_profile.py")
set_tests_properties(outdoor_profile PROPERTIES  _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;48;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
add_test(unload_zone "/home/tzb/nav_explore_ws/build/rover_state/test_unload_zone")
set_tests_properties(unload_zone PROPERTIES  _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;53;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
add_test(unload_alignment "/home/tzb/nav_explore_ws/build/rover_state/test_unload_alignment")
set_tests_properties(unload_alignment PROPERTIES  ENVIRONMENT "ROS_DOMAIN_ID=231;ROS_LOCALHOST_ONLY=1" _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;57;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
add_test(deadline_unload "/home/tzb/nav_explore_ws/build/rover_state/test_deadline_unload")
set_tests_properties(deadline_unload PROPERTIES  ENVIRONMENT "ROS_DOMAIN_ID=232;ROS_LOCALHOST_ONLY=1" _BACKTRACE_TRIPLES "/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;62;add_test;/home/tzb/nav_explore_ws/src/state/CMakeLists.txt;0;")
