# id of the initiatior who locked the topology entity
mars_common_msgs/Id initiator_id
# reason for logging
string reason
# The interval for the period from where the topology entity is locked
mars_topology_msgs/TimeInterval time_interval
# Id of the lock. Needed if you want to delete the lock.
mars_common_msgs/Id lock_id
