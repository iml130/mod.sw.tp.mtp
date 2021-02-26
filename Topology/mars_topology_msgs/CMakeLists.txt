cmake_minimum_required(VERSION 2.8.3)
project(mars_topology_msgs)

find_package(catkin REQUIRED
  message_generation
  std_msgs
  mars_common_msgs
  geometry_msgs
)

add_message_files(
  FILES
    TimeInterval.msg
    Value.msg
    Lock.msg
    Reservation.msg
    HazardType.msg
    Connection.msg
    TopologyEntity.msg
    TopologyEntityType.msg
    Registration.msg
    ReservationStamped.msg
    TopologyEntityRegistration.msg
    FullStatus.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   mars_common_msgs
   geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs mars_common_msgs geometry_msgs
)
