cmake_minimum_required(VERSION 2.8.3)
project(project1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        message_generation
        tf2
        tf2_ros
        )

add_message_files(
        FILES
        Rpm.msg
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

catkin_package(
        CATKIN_DEPENDS message_runtime
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
add_dependencies(wheel_speeds ${catkin_LIBRARIES})