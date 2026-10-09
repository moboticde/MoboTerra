cmake_minimum_required(VERSION 3.8)
project(mobotic_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mobotic_kinematics src/mobotic_kinematics.cpp)
target_include_directories(mobotic_kinematics PRIVATE include)
ament_target_dependencies(mobotic_kinematics
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(mobotic_kinematics Eigen3::Eigen)

install(TARGETS mobotic_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # No ROS runtime is needed for these production-cache regression tests.
  add_executable(test_joint_feedback_cache test/test_joint_feedback_cache.cpp)
  target_include_directories(test_joint_feedback_cache PRIVATE include)
  ament_add_test(test_joint_feedback_cache
    COMMAND "$<TARGET_FILE:test_joint_feedback_cache>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
