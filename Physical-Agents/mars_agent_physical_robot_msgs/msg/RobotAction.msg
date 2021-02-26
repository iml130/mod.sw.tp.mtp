# definition of all the possible actionsmars_agent_physical_robot_msgs
uint8 CATEGORY_UNDEFINED = 0
uint8 CATEGORY_NONE = 5
uint8 CATEGORY_LOAD = 10
uint8 CATEGORY_MANUAL_LOAD = 11
uint8 CATEGORY_MANUAL_LOAD_START = 12
uint8 CATEGORY_MANUAL_LOAD_DONE = 13
uint8 CATEGORY_UNLOAD = 20
uint8 CATEGORY_MANUAL_UNLOAD = 21
uint8 CATEGORY_MANUAL_UNLOAD_START = 22
uint8 CATEGORY_MANUAL_UNLOAD_DONE = 23
uint8 CATEGORY_START_CHARGING = 30
uint8 CATEGORY_STOP_CHARGING = 31
# ...


# Category of the action which has to be performed
uint8 category
# Defines the robot specific action which has to be performed. 
# The specific actions must be defined by manufacturer. 
uint8 action
mars_common_msgs/Tuple[] attributes
# Optional description of the action. E.g.: unload left
string description

