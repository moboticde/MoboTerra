cmake_minimum_required(VERSION 3.8)
project(sick_flexisoft_eth_comm)

# Find necessary ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(terraspiro_sick_interface REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED CONFIG COMPONENTS system)



add_executable(sick_flexisoft_eth_comm_node src/sick_flexisoft_eth_comm.cpp)
target_include_directories(sick_flexisoft_eth_comm_node
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(sick_flexisoft_eth_comm_node rclcpp std_msgs terraspiro_sick_interface)


# Install the executable, launch files, and config files
install(TARGETS
        sick_flexisoft_eth_comm_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)
# Finalize the ament build process
ament_package()
