cmake_minimum_required(VERSION 3.8)
project(terraspiro_vehicle_supervisor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(
        $<$<CONFIG:Release>:-O2>
)

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(terraspiro_sick_interface REQUIRED)

# Add the executable for the supervisor node
add_executable(terraspiro_vehicle_supervisor src/terraspiro_vehicle_supervisor.cpp)
target_include_directories(terraspiro_vehicle_supervisor
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# Specify include directories
ament_target_dependencies(terraspiro_vehicle_supervisor
        rclcpp
        std_srvs
        sensor_msgs
        geometry_msgs
        terraspiro_sick_interface)


# Install the executable
install(TARGETS
        terraspiro_vehicle_supervisor
        DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

# Install the header file
install(
        DIRECTORY include/
        DESTINATION include
)

ament_package()
