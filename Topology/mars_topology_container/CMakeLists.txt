cmake_minimum_required(VERSION 2.8.3)
project(mars_topology_container)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    std_msgs
    geometry_msgs
    mars_topology_msgs
    mars_topology_srvs
    mars_topology_actions
    actionlib_msgs
    mars_common
    mars_topology_edge
    mars_topology_vertex
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp
      actionlib
      std_msgs
      geometry_msgs
      mars_topology_msgs
      mars_common
      mars_topology_srvs
      mars_topology_actions
      actionlib_msgs
      mars_topology_edge
      mars_topology_vertex
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(./src SRC_LIST)

add_executable(${PROJECT_NAME}
  ${SRC_LIST} 
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roscpp mars_topology_srvs mars_common mars_topology_common)
  add_rostest_gtest(${PROJECT_NAME}_container_service_test test/testServices.test test/testServices.cpp)
  target_link_libraries(${PROJECT_NAME}_container_service_test ${catkin_LIBRARIES})

  find_package(catkin REQUIRED COMPONENTS rostest roscpp mars_topology_srvs mars_common mars_topology_common)
  add_rostest_gtest(${PROJECT_NAME}_container_action_test test/testActions.test test/testActions.cpp)
  target_link_libraries(${PROJECT_NAME}_container_action_test ${catkin_LIBRARIES})
endif()




