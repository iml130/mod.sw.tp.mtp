cmake_minimum_required(VERSION 2.8.3)
project(mars_agent_physical_common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mars_common
  mars_topology_common
  mars_agent_physical_robot_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mars_agent_physical_common
  CATKIN_DEPENDS roscpp mars_common mars_agent_physical_robot_msgs geometry_msgs mars_topology_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 aux_source_directory(./src SRC_LIST)

 add_library(${PROJECT_NAME} ${SRC_LIST}
 )


 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
