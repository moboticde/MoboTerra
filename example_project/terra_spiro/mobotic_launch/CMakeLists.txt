cmake_minimum_required(VERSION 3.14.4)
project(mobotic_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
            ../debug/can_rosbag_recorder.py
            scripts/circle_path_publisher.py
            scripts/line_path_publisher.py
            scripts/delete_bag_files.py
            scripts/twist_estimation_from_tf.py
            scripts/js_converter.py
            scripts/apriltag_localizer.py
            scripts/lidar_calibration.py
            scripts/relative_odom_loc_pose_calc.py
            scripts/world_tf_publisher.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(
        test_circle_path_stop
        test/test_circle_path_stop.py
    )
endif()

ament_package()
