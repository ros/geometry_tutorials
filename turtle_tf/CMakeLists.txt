cmake_minimum_required(VERSION 3.0.2)
project(turtle_tf)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy std_msgs tf turtlesim)

## Setup Include Directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Tell catkin what to export
catkin_package(CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf turtlesim)

## C++ examples
add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(turtle_tf_broadcaster ${catkin_EXPORTED_TARGETS})
endif()

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(turtle_tf_listener ${catkin_EXPORTED_TARGETS})
endif()

add_executable(turtle_tf_message_filter src/turtle_tf_message_filter.cpp)
target_link_libraries(turtle_tf_message_filter ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(turtle_tf_message_filter ${catkin_EXPORTED_TARGETS})
endif()

add_executable(turtle_tf_listener_debug src/turtle_tf_listener_debug.cpp)
target_link_libraries(turtle_tf_listener_debug ${catkin_LIBRARIES})


## Install Python Examples
install(PROGRAMS
  nodes/dynamic_tf_broadcaster.py
  nodes/fixed_tf_broadcaster.py
  nodes/turtle_tf_broadcaster.py
  nodes/turtle_tf_listener.py
  nodes/turtle_tf_listener_wait.py
  nodes/turtle_tf_message_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install C++ Examples
install(TARGETS turtle_tf_broadcaster turtle_tf_listener turtle_tf_message_filter turtle_tf_listener_debug
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
