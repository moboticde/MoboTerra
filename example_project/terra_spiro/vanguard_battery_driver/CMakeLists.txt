cmake_minimum_required(VERSION 3.13.0)
project(vanguard_battery_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(vanguard_driver src/vanguard_driver.cpp)
target_include_directories(vanguard_driver
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(vanguard_driver rclcpp std_msgs sensor_msgs ros2_socketcan std_srvs)

add_executable(virtual_vanguard_battery src/virtual_vanguard_battery.cpp)
target_include_directories(virtual_vanguard_battery
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(virtual_vanguard_battery rclcpp std_msgs sensor_msgs ros2_socketcan)

add_executable(vanguard_charge_monitor src/vanguard_charge_monitor.cpp)
target_include_directories(vanguard_charge_monitor
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(vanguard_charge_monitor rclcpp std_msgs std_srvs ros2_socketcan)

add_executable(vanguard_charge_rate_controller src/vanguard_charge_rate_controller.cpp)
target_include_directories(vanguard_charge_rate_controller
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(vanguard_charge_rate_controller rclcpp std_msgs ros2_socketcan)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(TARGETS
        vanguard_driver
        virtual_vanguard_battery
        vanguard_charge_monitor
        vanguard_charge_rate_controller
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_charge_rate_logic test/test_charge_rate_logic.cpp)
  target_include_directories(test_charge_rate_logic PRIVATE include)
  ament_target_dependencies(test_charge_rate_logic ros2_socketcan)
endif()


ament_package()
