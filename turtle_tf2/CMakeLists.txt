cmake_minimum_required(VERSION 3.0.2)
project(turtle_tf2)
## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  turtlesim
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtle_tf2
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## C++ examples
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
target_link_libraries(turtle_tf2_broadcaster ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(turtle_tf2_broadcaster ${catkin_EXPORTED_TARGETS})
endif()

add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
target_link_libraries(turtle_tf2_listener ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(turtle_tf2_listener ${catkin_EXPORTED_TARGETS})
endif()

add_executable(turtle_tf2_message_filter src/message_filter.cpp)
target_link_libraries(turtle_tf2_message_filter ${catkin_LIBRARIES})

## Install Python Examples
install(PROGRAMS
  nodes/dynamic_tf2_broadcaster.py
  nodes/fixed_tf2_broadcaster.py
  nodes/turtle_tf2_broadcaster.py
  nodes/turtle_tf2_listener.py
  nodes/turtle_tf2_listener_wait.py
  nodes/turtle_tf2_message_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install C++ Examples
install(TARGETS turtle_tf2_broadcaster turtle_tf2_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
