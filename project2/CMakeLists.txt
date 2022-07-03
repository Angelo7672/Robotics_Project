cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        roslaunch
)

catkin_package(

)

include_directories(
        include ${catkin_INCLUDE_DIRS}
)

add_executable(my_odom_tf src/odom_tf.cpp)
add_dependencies(my_odom_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_odom_tf ${catkin_LIBRARIES} )