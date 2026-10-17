cmake_minimum_required(VERSION 3.8)
project(patrol_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapBinding.msg"
  "msg/PatrolPoint.msg"
  "msg/RouteStep.msg"
  "msg/PatrolRoute.msg"
  "msg/PatrolStatus.msg"
  "msg/RunRecord.msg"
  "msg/OperatorHeartbeat.msg"
  "msg/ExecutorHeartbeat.msg"
  "srv/ListMaps.srv"
  "srv/RegisterMap.srv"
  "srv/SetActiveMap.srv"
  "srv/ListPoints.srv"
  "srv/UpsertPoint.srv"
  "srv/DeletePoint.srv"
  "srv/SaveCurrentPoint.srv"
  "srv/ListRoutes.srv"
  "srv/UpsertRoute.srv"
  "srv/DeleteRoute.srv"
  "srv/ControlRun.srv"
  "srv/GetRuns.srv"
  "srv/ResetInterlock.srv"
  "action/RunRoute.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
