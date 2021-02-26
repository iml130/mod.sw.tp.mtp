cmake_minimum_required(VERSION 2.8.3)
project(mars_topology_actions)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib
  std_msgs
  actionlib_msgs
  mars_common_msgs
  mars_topology_msgs
)

add_action_files( DIRECTORY action
   FILES
    AllocateEntity.action
)

generate_messages(
   DEPENDENCIES
     std_msgs
     actionlib_msgs
     mars_common_msgs
     mars_topology_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    actionlib 
    std_msgs 
    actionlib_msgs 
    mars_common_msgs
    mars_topology_msgs
)
