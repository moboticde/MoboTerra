cmake_minimum_required(VERSION 3.13.0)
project(mobotic_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/joint_controller.py
        scripts/tf_broadcaster.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()