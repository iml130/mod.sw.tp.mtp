cmake_minimum_required(VERSION 2.8.3)
project(mars_routing_plugin_carp)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  mars_routing_common
  mars_routing_core
  pluginlib
  mars_routing_srvs  
  mars_agent_physical_common
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib mars_routing_common mars_routing_core mars_routing_srvs mars_agent_physical_common
)

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/Router.cpp
  src/Route.cpp
  src/Step.cpp
  src/utilities.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(${PROJECT_NAME}-route-2x1-test test/Route2x1.test test/Route2x1.cpp)
  target_link_libraries(${PROJECT_NAME}-route-2x1-test ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}-route-3-3-test test/Route3-3.test test/Route3-3.cpp)
  target_link_libraries(${PROJECT_NAME}-route-3-3-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

