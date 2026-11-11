cmake_minimum_required(VERSION 3.13.0)
project(mobotic_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mobotic_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(mobotic_driver src/mobotic_driver.cpp)
target_include_directories(mobotic_driver
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(mobotic_driver
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        can_msgs
        ros2_socketcan
        std_srvs
        mobotic_interfaces)
ament_target_dependencies(mobotic_driver diagnostic_msgs)

add_executable(paced_socket_can_sender src/paced_socket_can_sender.cpp)
target_include_directories(paced_socket_can_sender PRIVATE include)
ament_target_dependencies(paced_socket_can_sender rclcpp can_msgs std_msgs)

add_executable(virtual_mobotic_drive src/virtual_mobotic_drive.cpp)
target_include_directories(virtual_mobotic_drive
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(virtual_mobotic_drive rclcpp std_msgs sensor_msgs geometry_msgs can_msgs ros2_socketcan)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(TARGETS
        mobotic_driver
        paced_socket_can_sender
        virtual_mobotic_drive
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_executable(test_micontrol_status test/test_micontrol_status.cpp)
  target_include_directories(test_micontrol_status PRIVATE include)
  ament_target_dependencies(test_micontrol_status can_msgs)
  ament_add_test(test_micontrol_status COMMAND "$<TARGET_FILE:test_micontrol_status>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  add_executable(test_can_scheduling test/test_can_scheduling.cpp)
  target_include_directories(test_can_scheduling PRIVATE include)
  ament_target_dependencies(test_can_scheduling can_msgs rclcpp std_msgs)
  ament_add_test(test_can_tx_queue COMMAND "$<TARGET_FILE:test_can_scheduling>" --queue
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  add_library(paced_sender_test_core OBJECT src/paced_socket_can_sender.cpp)
  target_compile_definitions(paced_sender_test_core PRIVATE MOBOTIC_PACED_CAN_SENDER_NO_MAIN)
  target_include_directories(paced_sender_test_core PRIVATE include)
  ament_target_dependencies(paced_sender_test_core rclcpp can_msgs std_msgs)
  ament_add_test(test_sdo_bus_scheduler COMMAND "$<TARGET_FILE:test_can_scheduling>" --bus
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_sdo_native_freshness COMMAND "$<TARGET_FILE:test_can_scheduling>" --freshness
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  add_executable(test_steering_sdo_updates test/test_steering_sdo_updates.cpp)
  target_include_directories(test_steering_sdo_updates PRIVATE include)
  ament_target_dependencies(test_steering_sdo_updates can_msgs rclcpp std_msgs)
  ament_add_test(test_steering_sdo_updates COMMAND "$<TARGET_FILE:test_steering_sdo_updates>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # ROS integration cases exercise the same driver implementation.
  add_library(driver_test_core OBJECT src/mobotic_driver.cpp)
  target_compile_definitions(driver_test_core PRIVATE MOBOTIC_DRIVER_NO_MAIN)
  target_compile_options(driver_test_core PRIVATE -O0)
  target_include_directories(driver_test_core PRIVATE include)
  ament_target_dependencies(driver_test_core rclcpp std_msgs sensor_msgs geometry_msgs can_msgs std_srvs mobotic_interfaces diagnostic_msgs)
  add_executable(test_micontrol_traction test/test_micontrol_traction.cpp
    $<TARGET_OBJECTS:driver_test_core> $<TARGET_OBJECTS:paced_sender_test_core>)
  target_compile_options(test_micontrol_traction PRIVATE -O0)
  target_include_directories(test_micontrol_traction PRIVATE include)
  ament_target_dependencies(test_micontrol_traction rclcpp std_msgs sensor_msgs geometry_msgs can_msgs std_srvs mobotic_interfaces diagnostic_msgs)
  ament_add_test(test_micontrol_traction COMMAND "$<TARGET_FILE:test_micontrol_traction>"
    --software-enable-diagnostics
    TIMEOUT 120 ENV ROS_DOMAIN_ID=152 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  ament_add_test(test_micontrol_traction_sdo_low_rate COMMAND "$<TARGET_FILE:test_micontrol_traction>" --feedback-period=0.1
    --software-enable-diagnostics
    TIMEOUT 120 ENV ROS_DOMAIN_ID=154 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  ament_add_test(test_micontrol_steering_restarts COMMAND "$<TARGET_FILE:test_micontrol_traction>"
    --feedback-period=0.1 --reply-delay-ms=2 --steering-updates-only --software-enable-diagnostics
    TIMEOUT 45 ENV ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  ament_add_test(test_micontrol_startup_preserves_position COMMAND "$<TARGET_FILE:test_micontrol_traction>"
    --feedback-period=0.1 --reply-delay-ms=2 --startup-hold-only --software-enable-diagnostics
    TIMEOUT 45 ENV ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  ament_add_test(test_micontrol_steering_alignment COMMAND "$<TARGET_FILE:test_micontrol_traction>"
    --feedback-period=0.1 --software-enable-diagnostics --align-on-enable
    TIMEOUT 60 ENV ROS_DOMAIN_ID=158 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_micontrol_nearest_home COMMAND "$<TARGET_FILE:test_micontrol_traction>"
    --feedback-period=0.1 --software-enable-diagnostics --align-on-enable --nearest-home
    TIMEOUT 60 ENV ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_micontrol_default_polling COMMAND "$<TARGET_FILE:test_micontrol_traction>" --idle-only
    TIMEOUT 15 ENV ROS_DOMAIN_ID=156 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_micontrol_sustained_freshness COMMAND "$<TARGET_FILE:test_micontrol_traction>"
    --feedback-period=0.1 --software-enable-diagnostics --reply-delay-ms=2 --sustained-only
    TIMEOUT 100 ENV ROS_DOMAIN_ID=157 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  add_executable(test_reset_recovery test/test_reset_recovery.cpp)
  target_include_directories(test_reset_recovery PRIVATE include)
  ament_add_test(test_reset_recovery COMMAND "$<TARGET_FILE:test_reset_recovery>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  add_executable(test_drive_startup test/test_drive_startup.cpp)
  target_include_directories(test_drive_startup PRIVATE include)
  ament_target_dependencies(test_drive_startup can_msgs)
  ament_add_test(test_drive_startup COMMAND "$<TARGET_FILE:test_drive_startup>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  add_executable(test_controller_protocol test/test_controller_protocol.cpp)
  target_include_directories(test_controller_protocol PRIVATE include)
  ament_target_dependencies(test_controller_protocol can_msgs rclcpp std_msgs)
  ament_add_test(test_controller_protocol COMMAND "$<TARGET_FILE:test_controller_protocol>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  add_executable(test_virtual_sdo test/test_virtual_sdo.cpp $<TARGET_OBJECTS:driver_test_core>)
  target_compile_options(test_virtual_sdo PRIVATE -O0)
  target_include_directories(test_virtual_sdo PRIVATE include)
  ament_target_dependencies(test_virtual_sdo rclcpp std_msgs sensor_msgs geometry_msgs can_msgs std_srvs mobotic_interfaces diagnostic_msgs)
  ament_add_test(test_virtual_sdo COMMAND "$<TARGET_FILE:test_virtual_sdo>"
    TIMEOUT 20 ENV ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # These run the real driver plus full controller models in one executor.
  # CPU contention between -O0 model executors is not a wire-load test and can
  # intentionally trip the unchanged command/lease freshness deadlines.
  set_tests_properties(test_micontrol_traction
    test_micontrol_traction_sdo_low_rate test_micontrol_default_polling
    test_micontrol_sustained_freshness test_micontrol_steering_alignment test_micontrol_nearest_home test_virtual_sdo
    test_micontrol_steering_restarts
    test_micontrol_startup_preserves_position
    PROPERTIES RUN_SERIAL TRUE)
  add_executable(test_update_protocol test/test_update_protocol.cpp)
  target_include_directories(test_update_protocol PRIVATE include)
  ament_target_dependencies(test_update_protocol can_msgs)
  ament_add_test(test_update_protocol
    COMMAND "$<TARGET_FILE:test_update_protocol>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(test_motion_source_gate test/test_motion_source_gate.cpp)
  target_include_directories(test_motion_source_gate PRIVATE include)
  ament_add_test(test_motion_source_gate
    COMMAND "$<TARGET_FILE:test_motion_source_gate>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_gtest(test_management_lease test/test_management_lease.cpp)
  target_include_directories(test_management_lease PRIVATE include)
  ament_add_gtest(test_feedback_freshness test/test_feedback_freshness.cpp)
  target_include_directories(test_feedback_freshness PRIVATE include)
  ament_target_dependencies(test_feedback_freshness can_msgs rclcpp)
  add_executable(test_can_feedback_ordering test/test_can_feedback_ordering.cpp)
  target_include_directories(test_can_feedback_ordering PRIVATE include)
  # ROS builds use real can_msgs. The standalone adapter is intentionally NOT included.
  ament_target_dependencies(test_can_feedback_ordering can_msgs)
  ament_add_test(test_can_feedback_ordering
    COMMAND "$<TARGET_FILE:test_can_feedback_ordering>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_steering_target_limits test/test_steering_target_limits.cpp)
  target_include_directories(test_steering_target_limits PRIVATE include)
  ament_target_dependencies(test_steering_target_limits can_msgs)
  ament_add_test(test_steering_target_limits
    COMMAND "$<TARGET_FILE:test_steering_target_limits>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_virtual_drive_protocol test/test_virtual_drive_protocol.cpp)
  target_include_directories(test_virtual_drive_protocol PRIVATE include)
  ament_target_dependencies(test_virtual_drive_protocol can_msgs)
  ament_add_test(test_virtual_drive_protocol
    COMMAND "$<TARGET_FILE:test_virtual_drive_protocol>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
