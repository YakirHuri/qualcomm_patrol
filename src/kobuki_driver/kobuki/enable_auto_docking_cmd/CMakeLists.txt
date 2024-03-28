cmake_minimum_required(VERSION 2.8.3)
project(enable_auto_docking_cmd)


add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-ffunction-sections -fdata-sections")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  kobuki_msgs
)

include_directories(${catkin_INCLUDE_DIRS})



catkin_package()


add_executable(enable_auto_docking_cmd_node
  src/enable_auto_docking_cmd_node.cpp
)

add_dependencies(enable_auto_docking_cmd_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(enable_auto_docking_cmd_node
  ${catkin_LIBRARIES}

)
