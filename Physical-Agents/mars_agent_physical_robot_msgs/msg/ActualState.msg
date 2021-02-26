# Message for the Actual State
std_msgs/Header header
mars_common_msgs/Id robot_id
# Definition of all possible States
uint8 MBS_NO_POSITION = 0
uint8 MBS_WAITING_FOR_INIT = 1
uint8 MBS_READY = 2
uint8 MBS_FIRST_ROTATION = 3
uint8 MBS_MOVING = 4
uint8 MBS_SECOND_ROTATION = 5
uint8 MBS_FINISHED = 6
uint8 MBS_FETCH_NEXT_INSTRUCTION = 7
uint8 MBS_EXECUTE_ACTION = 8
uint8 MBS_OBSTACLE = 9
uint8 MBS_ERROR = 10
uint8 MoveBaseSimpleState
