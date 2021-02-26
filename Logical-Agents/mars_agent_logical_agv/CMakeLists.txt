cmake_minimum_required(VERSION 2.8.3)
project(mars_agent_logical_agv)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mars_common
  mars_routing_common
  mars_agent_logical_srvs
  mars_agent_logical_common
  behaviortree_cpp_v3
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp std_msgs
                  mars_common
                  mars_routing_common
                  mars_agent_logical_srvs
                  mars_agent_logical_common
                  behaviortree_cpp_v3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(./src SRC_LIST)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
