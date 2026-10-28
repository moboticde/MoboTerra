cmake_minimum_required(VERSION 3.8)
project(kinematics_conversions)

# Find necessary ROS 2 and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

# C++ Executable: joint_state_to_twist
add_executable(joint_state_to_twist src/joint_state_to_twist.cpp)
ament_target_dependencies(joint_state_to_twist
        rclcpp
        geometry_msgs
        sensor_msgs
        std_srvs
)
target_include_directories(joint_state_to_twist
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(joint_state_to_twist Eigen3::Eigen)

# C++ Executable: twist_to_acker
add_executable(twist_to_acker src/twist_to_acker.cpp include/kinematics_conversions/twist_to_acker.h)
target_include_directories(twist_to_acker PUBLIC include)
ament_target_dependencies(twist_to_acker
        rclcpp
        geometry_msgs
        ackermann_msgs
        std_srvs
)

add_executable(inverse_kinematics_node
        src/inverse_kinematics_node.cpp)
target_include_directories(inverse_kinematics_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(inverse_kinematics_node rclcpp sensor_msgs geometry_msgs std_srvs)

install(TARGETS twist_to_acker joint_state_to_twist inverse_kinematics_node
        DESTINATION lib/${PROJECT_NAME}
)

# Python script installation
install(PROGRAMS scripts/joint_states_to_acker.py
        DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
        DESTINATION include
)

# Install launch and config files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

# Standard ament package call at the end
ament_package()