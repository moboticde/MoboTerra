cmake_minimum_required(VERSION 3.8)
project(teltonika_gps_server)

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
find_package(Boost REQUIRED)
find_package(nmea REQUIRED)

# Add the executable for the supervisor node
add_executable(teltonika_gps_server src/teltonika_gps_server.cpp)
target_include_directories(teltonika_gps_server
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(teltonika_gps_server nmea)
# Specify include directories
ament_target_dependencies(teltonika_gps_server
        rclcpp
        std_srvs
        sensor_msgs
        geometry_msgs
        Boost
        nmea)


# --- Add the NMEA Client Executable ---
add_executable(nmea_client_node src/nmea_client_node.cpp)
# The client does not use ROS libraries, only standard POSIX/C++ libs

install(TARGETS
        nmea_client_node
        DESTINATION lib/${PROJECT_NAME}
)


# Install the executable
install(TARGETS
        teltonika_gps_server
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
