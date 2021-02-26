# Introduction 
Short introduction of the node! 

* Maintainer: 
    * Mark Down <mark.down@iml.fraunhofer.de>
    * Max Mustermann <max.mustermann@iml.fraunhofer.de>
* Main author: Mark Down, <mark.down@iml.fraunhofer.de>

## Nodes
* MarsTopologyEdge

## Expected behaviour
At start the node reads all given parameteres from the parameter server in order to initialize itself.
After Initialization the node mostly reacts to service calls, while maintaining a consistent list of reservations of itself from  other agents (vehicles).
The only exceptions to this are:
* Allocation action which gives the vehicle feedback to its current position in the waiting queue
* Visualization of itself based on its footprint

# Provided Interactions
Interactions which are provided by the ros node. 

## Published Topics

* **registration** (mars_topology_msgs/Reservation.msg)
    
    Registers itself at the MARS to declare itself ready for action.

* **visualization_footprint** ([visualization_msgs/Marker.msg](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html))
    
    Visualizes the entity based on its footprint

* **visualization_direction** ([visualization_msgs/Marker.msg](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html))
    
    Visualizes the direction of the arrow.

## Services

* **get_coordinate** (mars_topology_srvs/GetCoordinate.srv)
    
    Gets the coordinate of the entity.

* **get_footprint** (mars_topology_srvs/GetFootprint.srv)
    
    Gets the footprint of the entity.

* **lock** (mars_topology_srvs/LockTopologyEntity.srv)(**TODO**)
    
    Locks the entity for a given time.

* **unlock** (mars_topology_srvs/UnlockTopologyEntity.srv)(**TODO**)
    
    Unlocks the entity for given time.

* **add_reservation** (mars_topology_srvs/AddReservation.srv)
    
    Adds a reservation as long as it does not collide with another reservation.

* **deallocate** (mars_topology_srvs/DeallocateEntity.srv)
    
    Deallocates the entity, which means that the vehicle loses its exclusive rights to drive on this entity.

* **delete_reservation** (mars_topology_srvs/DeleteReservation.srv)
    
    Deletes a reservation.

* **get_free_time_slots** (mars_topology_srvs/GetFreeTimeSlots.srv)
    
    Gets aviable free time slots for possible reservations.

* **get_connections** (mars_topology_srvs/GetConnections.srv)
    
    Gets the origin and destination of this edge.

* **get_restrictions** (mars_topology_srvs/GetResstrictions.srv)
    
    Gets resrictions of this entity, for example max velocity allowed.

* **get_type** (mars_topology_srvs/GetType.srv)
    
    Gets the type of this entity.

* **get_status** (mars_topology_srvs/GetStatus.srv)
    
    Gets the current status of this entity.
    
* **get_length** (mars_topology_srvs/GetLength.srv)
    
    Gets the length of this edge.

## Action API

* **allocate** (mars_topology_actions/AllocateEntity) 

    Used to allocate the topology entity by an vehicle in order to gain exclusive driving rights 

# Parameters

## Mandatory Parameters
* **~id** (string) 

    The uuid of this entity.

* **~type** (int) 

    The type of the entity.
    
* **~direction** (int) 

    The direction of this edge.
    
* **~origin_id** (string) 

    The uuid of the origin vertex of this edge.
    
* **~destination_id** (string) 

    The uuid of the destination vertex of this edge.
    
* **~is_locked** (bool) 

    The type of the entity.
    
* **~length** (float)

    The length of this edge.

* **~x_pos** (float)

    The x coordinate of this entity.
    
* **~y_pos** (float)

    The y coordinate of this entity.
    
* **~footprint_x** (float array) 

    The x coordinates of the footprint. Can be left empty, if footprint_y is also empty.
    
* **~footprint_y** (float array) 

    The y coordinates of the footprint. Can be left empty, if footprint_x is also empty.
    
## Optional Parameters
* **~log_level** (int, default: "debug") 

    Log level of this node.
    
* **~frame_id** (string, default: "topology")

    The TF2 frame id.
    
* **~node_rate** (int, default: 10)

    The ROS spin rate of the node in Hz.
    
* **~visualization_rate** (float, default: 0.1)
    
    The rate of the node visualization in Hz.

* **~description** (string, default: "") 

    Short description of this entity to improve readability for humans.
    
* **~service_name_get_coordinate** (string, default: "get_coordinate") 

    Name of the service, which gets the entity coordinates.
    
* **~service_name_get_footprint** (string, default: "get_footprint") 

    Name of the service, which gets the entity footprint.
    
* **~service_name_lock** (string, default: "lock") 

    Name of the service, which locks the entity.
    
* **~service_name_unlock** (string, default: "unlock") 

    Name of the service, which unlocks the entity.
    
* **~service_name_get_length** (string, default: "get_length") 

    Name of the service, which gets the length of this edge.
    
* **~service_name_get_connections** (string, default: "get_connections") 

    Name of the service, which gets the origin and destination of this edge.
    
* **~service_name_add_reservation** (string, default: "add_reservation") 

    Name of the service, which adds a reservation to the entity.
    
* **~service_name_deallocate** (string, default: "deallocate") 

    Name of the service, which deallocates the entity.
    
* **~service_name_delete_reservation** (string, default: "delete_reservation") 

    Name of the service, which deletes a reservation from the entity.
    
* **~service_name_get_free_time_slots** (string, default: "get_free_time_slots") 

    Name of the service, which gets the free time slots from the entity.
    
* **~service_name_get_ingoing_edges** (string, default: "get_ingoing_edges") 

    Name of the service, which gets the ingoing edges from the entity.
    
* **~service_name_get_outgoing_edges** (string, default: "get_outgoing_edges") 

    Name of the service, which gets the outgoing edges from the entity.
    
* **~service_name_get_restrictions** (string, default: "get_restrictions") 

    Name of the service, which gets the restrictions of the entity.
    
* **~service_name_get_type** (string, default: "get_type") 

    Name of the service, which gets the type of the entity.
    
* **~service_name_get_status** (string, default: "get_status") 

    Name of the service, which gets the status of the entity.
    
* **~action_name_allocate** (string, default: "allocate") 

    Name of the action, which lets vehicle gain exclusive driving rights on this entity.
    
* **~topic_name_register** (string, default: "register") 

    Name of the published topic, which registrates the entity on the mars system.
    
* **~topic_name_visualization_footprint** (string, default: "visualization_footprint") 

    Name of the published topic, which visualizes the entity based on its footprint.
    
* **~topic_name_visualization_direction** (string, default: "visualization_direction") 

    Name of the published topic, which visualizes the direction of this edge.
    
* **~maximum_linear_velocity** (int, default: "10") 

    Max linear velocity allowed to drive over this entity in m/s.
    
* **~maximum_angular_velocity** (int, default: "1") 

    Max angular velocity allowed to drive over this entity in rad/s.
    
* **~maximum_linear_acceleration** (int, default: "1") 

    Max linear acceleration allowed to drive over this entity in m/s².
    
* **~maximum_angular_acceleration** (int, default: "0.5") 

    Max angular acceleration allowed to drive over this entity in rad/s².
    
* **~maximum_height** (int, default: "2") 

    Max height of vehicles which are allowed on this entity in m.
    
* **~maximum_total_weight** (int, default: "200") 

    Max weight of vehicles which are allowed on this entity in kg.

* **~forbidden_vehicle_types** (**TODO**, default: "**TODO**") 

    **TODO**
    
* **~forbidden_hazard_types** (**TODO**, default: "**TODO**") 

    **TODO**