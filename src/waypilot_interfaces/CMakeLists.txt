cmake_minimum_required(VERSION 3.8)
project(waypilot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/MissionStatus.msg"
  "srv/AddWaypoint.srv"
  "srv/RemoveWaypoint.srv"
  "srv/MissionCommand.srv"
  "action/NavigateToPoint.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
