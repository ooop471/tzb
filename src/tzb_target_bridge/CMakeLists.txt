cmake_minimum_required(VERSION 3.8)
project(tzb_target_bridge)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 自定义消息：目标物体在 base_frame 下的位置与方位
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetInBase.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Python 节点
install(PROGRAMS
  scripts/target_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
