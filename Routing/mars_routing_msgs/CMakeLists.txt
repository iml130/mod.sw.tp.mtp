cmake_minimum_required(VERSION 2.8.3)
project(mars_routing_msgs)

find_package(catkin REQUIRED
  message_generation
  std_msgs
  geometry_msgs
  mars_common_msgs
  mars_topology_msgs
  mars_agent_physical_robot_msgs
)

add_message_files(
  FILES
    TimeProfile.msg
    Step.msg
    Route.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   mars_common_msgs
   mars_topology_msgs
   geometry_msgs
   mars_agent_physical_robot_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs mars_common_msgs mars_topology_msgs mars_agent_physical_robot_msgs
)
