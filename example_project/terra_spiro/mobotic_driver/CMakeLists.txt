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

add_executable(mobotic_driver src/mobotic_driver.cpp)
target_include_directories(mobotic_driver
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(mobotic_driver rclcpp std_msgs sensor_msgs geometry_msgs can_msgs ros2_socketcan std_srvs)

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


ament_package()
