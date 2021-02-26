cmake_minimum_required(VERSION 2.8.3)
project(mars_agent_physical_robot_msgs)

find_package(catkin REQUIRED
  message_generation
  std_msgs
  geometry_msgs
  industrial_msgs
  sensor_msgs
  mars_common_msgs
)

add_message_files(
  FILES
    Sequence.msg
    ActionAssignment.msg
    CancelTask.msg
    MotionAssignment.msg
    RobotAgentProperties.msg
    VehicleType.msg
    ActualState.msg
    RobotAction.msg
    AssignmentStatus.msg
    Motion.msg
)

generate_messages(
   DEPENDENCIES
    std_msgs
    geometry_msgs
    industrial_msgs
    sensor_msgs
    mars_common_msgs
)
catkin_package(
   CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    industrial_msgs
    sensor_msgs
    mars_common_msgs
)
