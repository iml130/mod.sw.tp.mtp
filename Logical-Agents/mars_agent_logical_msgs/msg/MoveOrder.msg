# The id of the transport order where this transport order step belongs to
mars_common_msgs/Id move_order_id

# The destination entity
mars_topology_msgs/TopologyEntity destination_entity

# The estimated time for which the destination entity is occupied.
# Please only set this parameter when the time will deviate from default time.
duration destination_reservation_time
