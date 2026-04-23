cmake_minimum_required(VERSION 3.8)
project(scout_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvoidanceStatus.msg"
  "msg/DroneHealth.msg"
  "msg/DroneReadiness.msg"
  "msg/PeerTelemetry.msg"
  "msg/TargetCommand.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
