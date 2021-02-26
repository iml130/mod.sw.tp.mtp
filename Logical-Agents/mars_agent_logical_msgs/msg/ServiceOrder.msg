# Definition of existing service order derivations
# Maintenance service order
uint8 MAINTENANCE_SERVICE_ORDER = 0

# The type of the move order
uint8 type

# The id of the start vertex 
mars_agent_logical_msgs/MoveOrder move_order

# The estimated time for which the destination vertex is occupied
duration destination_reservation_time