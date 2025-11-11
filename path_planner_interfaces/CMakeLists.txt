cmake_minimum_required(VERSION 3.8)
project(path_planner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InitiatePath.srv"
  "msg/Sphere.msg"
  "msg/AABB.msg"
  "msg/OrientedBox.msg"
  "srv/SetObstacles.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
