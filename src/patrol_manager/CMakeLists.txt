cmake_minimum_required(VERSION 2.8.3)
project(patrol_manager)

find_package(OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  tf
  nav_msgs
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${OpenCV_INCLUDE_DIRS}

)

add_definitions(-std=c++11)



add_executable(patrol_manager_node 
    src/patrol_manager_node.cpp
    src/MoveBaseController.cpp
)
target_link_libraries(patrol_manager_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}

)


add_dependencies(patrol_manager_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)


#############
## Install ##
#############

install(TARGETS patrol_manager_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

