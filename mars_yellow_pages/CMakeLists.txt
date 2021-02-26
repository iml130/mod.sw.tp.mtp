cmake_minimum_required(VERSION 2.8.3)
project(mars_yellow_pages)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mars_topology_msgs
    mars_topology_srvs
    mars_common
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES  mars_yellow_pages
    CATKIN_DEPENDS
      roscpp
      mars_common
      mars_topology_msgs
      mars_topology_srvs
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

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


