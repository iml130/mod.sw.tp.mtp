cmake_minimum_required(VERSION 2.8.3)
project(mars_routing_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  mars_common_msgs
  mars_topology_msgs
  mars_routing_msgs
  mars_agent_physical_robot_msgs
)

add_service_files(
   FILES
   GetRoute.srv
   RemoveRouteVisualization.srv
   SetRouteVisualizationStartTime.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   mars_common_msgs
   mars_routing_msgs
   mars_topology_msgs
   mars_agent_physical_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs mars_common_msgs mars_routing_msgs mars_agent_physical_robot_msgs mars_topology_msgs
)
