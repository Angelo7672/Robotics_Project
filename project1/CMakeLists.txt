cmake_minimum_required(VERSION 3.0.2)
project(project1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        message_generation
        tf2
        tf2_ros
        dynamic_reconfigure
        message_filters
        )

add_message_files(
        FILES
        Rpm.msg
        Calibration.msg
)

add_service_files(
        FILES
        Reset.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
)

generate_dynamic_reconfigure_options(
        cfg/parameters.cfg
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        message_runtime
        geometry_msgs
        nav_msgs
        sensor_msgs
        message_filters
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(reset_client src/client.cpp)
target_link_libraries(reset_client ${catkin_LIBRARIES})
add_dependencies(reset_client ${catkin_EXPORTED_TARGETS})

add_executable(kinematics src/kinematics.cpp)
target_link_libraries(kinematics ${catkin_LIBRARIES})
add_dependencies(kinematics ${catkin_EXPORTED_TARGETS})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})

add_executable(wheel_speeds src/wheel_speeds.cpp)
target_link_libraries(wheel_speeds ${catkin_LIBRARIES})
add_dependencies(wheel_speeds ${catkin_EXPORTED_TARGETS})

add_executable(calibrator src/estimator_for_calibration.cpp)
target_link_libraries(calibrator ${catkin_LIBRARIES})
add_dependencies(calibrator ${catkin_EXPORTED_TARGETS})