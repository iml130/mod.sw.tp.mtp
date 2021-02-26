cmake_minimum_required(VERSION 2.8.3)
project(mars_routing_common)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_eigen
  mars_common
  mars_routing_srvs
  mars_topology_srvs
  mars_topology_msgs
  mars_topology_common
  mars_topology_actions
  mars_agent_physical_common
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mars_routing_common
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_eigen
    mars_common
    mars_routing_srvs
    mars_topology_srvs
    mars_topology_msgs
    mars_topology_common
    mars_topology_actions
    mars_agent_physical_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

 aux_source_directory(./src SRC_LIST)
 aux_source_directory(./src/geometry SRC_LIST)
 aux_source_directory(./src/topology SRC_LIST)
 aux_source_directory(./src/utility SRC_LIST)
 aux_source_directory(./include/mars_routing_common/Cache.h SRC_LIST)

 add_library(${PROJECT_NAME} ${SRC_LIST}
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}-geometry-utilities-test test/geometry/utilities.cpp)
  target_link_libraries(${PROJECT_NAME}-geometry-utilities-test ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}-topology-vertex-test test/topology/Vertex.test test/topology/Vertex.cpp)
  target_link_libraries(${PROJECT_NAME}-topology-vertex-test ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}-topology-edge-test test/topology/Edge.test test/topology/Edge.cpp)
  target_link_libraries(${PROJECT_NAME}-topology-edge-test ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}-utility-timeprediction-test test/utility/timeprediction.test test/utility/timeprediction.cpp)
  target_link_libraries(${PROJECT_NAME}-utility-timeprediction-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
