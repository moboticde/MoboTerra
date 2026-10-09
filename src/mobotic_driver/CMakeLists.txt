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
        virtual_mobotic_drive
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
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
