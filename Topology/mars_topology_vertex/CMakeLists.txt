cmake_minimum_required(VERSION 2.8.3)
project(mars_topology_vertex)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
    actionlib
    std_msgs
    geometry_msgs
    mars_topology_common
    mars_topology_msgs
    mars_topology_srvs
    mars_topology_actions
    actionlib_msgs
    mars_common
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES  mars_topology_vertex
    CATKIN_DEPENDS
      roscpp
      tf2
      tf2_ros
      actionlib
      std_msgs
      geometry_msgs
      mars_topology_msgs
      mars_common
      mars_topology_srvs
      mars_topology_actions
      mars_topology_common
      actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(./src SRC_LIST)

SET(SRC
  src/MarsVertex.cpp
)
add_library(${PROJECT_NAME} ${SRC})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roscpp mars_topology_srvs mars_common mars_topology_common)
  add_rostest_gtest(${PROJECT_NAME}-vertex-service-test test/testServices.test test/testServices.cpp)
  target_link_libraries(${PROJECT_NAME}-vertex-service-test ${catkin_LIBRARIES})

  find_package(catkin REQUIRED COMPONENTS rostest roscpp mars_topology_srvs mars_common mars_topology_common)
  add_rostest_gtest(${PROJECT_NAME}-vertex-action-test test/testActions.test test/testActions.cpp)
  target_link_libraries(${PROJECT_NAME}-vertex-action-test ${catkin_LIBRARIES})

endif()
