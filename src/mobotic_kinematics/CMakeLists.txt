cmake_minimum_required(VERSION 3.8)
project(mobotic_kinematics)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/romb_kinematics scripts/romb_kinematics_adapter
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(test_name test_romb_kinematics_contract test_steering_command_cadence test_exclusive_manual_steering)
    ament_add_pytest_test(${test_name} test/${test_name}.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()
  ament_add_pytest_test(test_romb_solver_ros test/test_romb_solver_ros.py
    TIMEOUT 45 APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1)
  set_tests_properties(test_romb_solver_ros PROPERTIES RUN_SERIAL TRUE)
endif()

ament_package()
