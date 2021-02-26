cmake_minimum_required(VERSION 2.8.3)
project(mars_agent_logical_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  mars_common_msgs
  mars_agent_logical_msgs
  mars_topology_msgs
)

add_service_files(
   FILES
    AddTransportOrder.srv
    AddServiceOrder.srv
    AddMoveOrder.srv
    AddTransportOrderStep.srv
    DeleteOrder.srv
    DeleteOrderQueue.srv
    SetMode.srv
    GetOrderBid.srv
    ManualActionDone.srv
)

generate_messages(
   DEPENDENCIES
     std_msgs
     mars_common_msgs
     mars_agent_logical_msgs
     mars_topology_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs mars_common_msgs mars_agent_logical_msgs mars_topology_msgs
)
