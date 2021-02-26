cmake_minimum_required(VERSION 2.8.3)
project(mars_common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mars_common_msgs
  mars_topology_msgs
  visualization_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mars_common
  CATKIN_DEPENDS
    roscpp
    rospy
    geometry_msgs
    mars_common_msgs
    mars_topology_msgs
    visualization_msgs
    std_msgs
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
catkin_add_gtest(${PROJECT_NAME}-id-test test/Id.cpp)
target_link_libraries(${PROJECT_NAME}-id-test ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-time-interval-test test/TimeInterval.cpp)
target_link_libraries(${PROJECT_NAME}-time-interval-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
