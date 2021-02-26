cmake_minimum_required(VERSION 2.8.3)
project(mars_topology_launcher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  mars_topology_srvs
  message_generation
  mars_common
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
#  LIBRARIES mars_topology_launcher
  CATKIN_DEPENDS rospy mars_topology_srvs mars_common
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
