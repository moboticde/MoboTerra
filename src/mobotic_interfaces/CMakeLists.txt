cmake_minimum_required(VERSION 3.8)
project(mobotic_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyIOState.msg"
  "msg/SafetyState.msg"
  "msg/ManualControlState.msg"
  "msg/OperatorState.msg"
  "msg/DriveStatus.msg"
  "msg/ScannerSafetyState.msg"
  "msg/SafetyTelegram.msg"
  "msg/ComponentHealth.msg"
  "msg/ComponentHealthArray.msg"
  "msg/WheelModuleCommandArray.msg"
  "msg/WheelModuleOperationCommand.msg"
  "msg/BatterySystemState.msg"
  "msg/VehicleModeRequest.msg"
  "msg/VehicleModeState.msg"
  "msg/WheelModuleCommand.msg"
  "msg/WheelModuleStatus.msg"
  "msg/WheelModuleStatusArray.msg"
  "srv/SetVehicleMode.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
