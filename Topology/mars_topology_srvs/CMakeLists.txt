cmake_minimum_required(VERSION 2.8.3)
project(mars_topology_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  mars_common_msgs
  mars_topology_msgs
  mars_agent_physical_robot_msgs
)

add_service_files(
   FILES
    AddReservation.srv
    GetFootprint.srv
    GetRestrictions.srv
    DeallocateEntity.srv
    GetFreeTimeSlots.srv
    GetType.srv
    DeleteReservation.srv
    GetIngoingEdges.srv
    LockTopologyEntity.srv
    GetCoordinate.srv
    GetOutgoingEdges.srv
    UnlockTopologyEntity.srv
    GetLength.srv
    GetStatus.srv
    GetFullStatus.srv
    GetConnections.srv
    GetFootprint.srv
    GetTopologyEntityContainerId.srv
)

generate_messages(
   DEPENDENCIES
     std_msgs
     geometry_msgs
     mars_common_msgs
     mars_topology_msgs
     mars_agent_physical_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs mars_common_msgs mars_topology_msgs mars_agent_physical_robot_msgs
)
