cmake_minimum_required(VERSION 2.8.3)
project(mars_topology_common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mars_common
  mars_common_msgs
  mars_topology_msgs
  mars_agent_physical_robot_msgs
  mars_topology_srvs
  mars_topology_actions
  visualization_msgs
  tf2_geometry_msgs
  interactive_markers
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mars_topology_common
  CATKIN_DEPENDS roscpp mars_common mars_common_msgs mars_topology_msgs mars_agent_physical_robot_msgs mars_topology_srvs mars_topology_actions visualization_msgs tf2_geometry_msgs interactive_markers 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 aux_source_directory(./src SRC_LIST)

 add_library(${PROJECT_NAME} ${SRC_LIST}
 )

 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 if (CATKIN_ENABLE_TESTING)
 find_package(catkin REQUIRED COMPONENTS rostest roscpp mars_common)
  add_rostest_gtest(${PROJECT_NAME}-topology-entity-test test/TopologyEntity.test test/TopologyEntity.cpp)
  target_link_libraries(${PROJECT_NAME}-topology-entity-test ${PROJECT_NAME} ${catkin_LIBRARIES})
 endif()
