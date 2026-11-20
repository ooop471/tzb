cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake_auto REQUIRED)

install(PROGRAMS scripts/state_visual_status.py DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  config launch rviz behavior_tree
)
