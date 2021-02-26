cmake_minimum_required(VERSION 2.8.3)
project(mars_routing_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  rostest
  roscpp
  mars_routing_common
  mars_routing_core
  mars_common
  mars_routing_srvs
  mars_topology_msgs
  mars_common_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pluginlib roscpp mars_routing_common mars_routing_core mars_common mars_routing_srvs mars_topology_msgs mars_common_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

aux_source_directory(./src SRC_LIST)

add_executable(${PROJECT_NAME}_node ${SRC_LIST})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(${PROJECT_NAME}-carp-3-3-test test/Carp3-3.test test/Carp3-3.cpp)
  target_link_libraries(${PROJECT_NAME}-carp-3-3-test ${catkin_LIBRARIES})
endif()
