cmake_minimum_required(VERSION 2.8.3)
project(turtle_tf)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})
add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
