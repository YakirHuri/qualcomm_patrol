cmake_minimum_required(VERSION 2.8.3)
project(waypoints_markers_publishers)

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

add_executable(waypoints_markers_publishers_node 
    src/waypoints_markers_publishers_node.cpp
)

add_dependencies(waypoints_markers_publishers_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(waypoints_markers_publishers_node
  ${catkin_LIBRARIES}
)
