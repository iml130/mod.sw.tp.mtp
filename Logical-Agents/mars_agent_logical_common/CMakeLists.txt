cmake_minimum_required(VERSION 2.8.3)
project(mars_agent_logical_common)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  behaviortree_cpp_v3
  geometry_msgs
  mars_common
  mars_topology_msgs
  mars_agent_logical_msgs
  mars_agent_logical_srvs
  mars_routing_core
  mars_routing_common
  mars_topology_common
  mars_agent_physical_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mars_agent_logical_common
  CATKIN_DEPENDS roscpp behaviortree_cpp_v3 geometry_msgs mars_common mars_topology_msgs mars_agent_logical_msgs mars_agent_logical_srvs mars_routing_core mars_routing_common mars_topology_common mars_agent_physical_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 aux_source_directory(./src SRC_LIST)
 aux_source_directory(./src/behavior SRC_LIST)
 aux_source_directory(./src/behavior/OrderStatus SRC_LIST)
 aux_source_directory(./src/behavior/Decorators SRC_LIST)
 aux_source_directory(./src/behavior/Conditions SRC_LIST)

 add_library(${PROJECT_NAME} ${SRC_LIST}
 )

 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
