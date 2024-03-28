cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_laser_navigation)

find_package(catkin REQUIRED COMPONENTS
  #amcl
  #hector_mapping
  move_base
  move_base_msgs
  actionlib
  tf
  kobuki_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_definitions(-std=c++11)

add_executable(waypoint_driver_node 
    src/waypoint_driver_node.cpp
    src/WaypointDriver.cpp
    src/MoveBaseController.cpp
)

add_dependencies(waypoint_driver_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(waypoint_driver_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#install(TARGETS waypoint_driver_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
