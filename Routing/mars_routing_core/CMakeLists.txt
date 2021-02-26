cmake_minimum_required(VERSION 2.8.3)
project(mars_routing_core)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  mars_routing_common
  mars_routing_srvs
  mars_common
  mars_topology_msgs
  mars_common_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mars_routing_core
  CATKIN_DEPENDS roscpp eigen_conversions mars_common mars_routing_common mars_routing_srvs mars_topology_msgs mars_common_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

aux_source_directory(./src SRC_LIST)

add_library(${PROJECT_NAME} ${SRC_LIST})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

if (CATKIN_ENABLE_TESTING)
catkin_add_gtest(${PROJECT_NAME}-step-test test/Step.cpp)
target_link_libraries(${PROJECT_NAME}-step-test ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-route-test test/Route.cpp)
target_link_libraries(${PROJECT_NAME}-route-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
