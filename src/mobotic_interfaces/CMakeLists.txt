cmake_minimum_required(VERSION 3.8)
project(mobotic_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyIOState.msg"
  "msg/SafetyState.msg"
  "msg/ManualControlState.msg"
  "msg/BatterySystemState.msg"
  "msg/VehicleModeRequest.msg"
  "msg/VehicleModeState.msg"
  "msg/WheelModuleCommand.msg"
  "msg/WheelModuleStatus.msg"
  "msg/WheelModuleStatusArray.msg"
  "srv/SetVehicleMode.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
