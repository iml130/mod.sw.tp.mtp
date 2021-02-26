## Published Topics from physical agent
* *current_motion* (mars_agent_physical_robot_msgs/Motion)

    Publishes the current pose and velocity of the vehicle.

* *robot_status* (industrial_msgs/RobotStatus)
    
    Publishes the current internal status of the vehicle/motors

* *battery_state* (sensor_msgs/BatteryState)
    
    Publishes the current status of the battery

* *actual_state* (mars_agent_physical_robot_msgs/ActualState)
    
    Publishes the current state of the internal state machine. (Mostly for debug reasons)

* *robot_properties* (mars_agent_physical_robot_msgs/RobotAgentProperties)
    
    Publishes the description of the vehicle, which includes the limitations of the specific vehicle.

* *current_assignment_status* (mars_agent_physical_robot_msgs/AssignmentStatus)
    
    Publishes the current executed order, assignments and footprint referenced by its ID.

## Subscribed Topics from physical agent

* *action_assignment* (mars_agent_physical_robot_msgs/ActionAssignment)
    
    Subscribes to the next action assignment from the logical agent.

* *motion_assignment* (mars_agent_physical_robot_msgs/MotionAssignment)
    
    Subscribes to the next motion assignment from the logical agent.

* *cancel_order* (mars_agent_physical_robot_msgs/CancelOrder)
    
    Tells which order should be deleted. 

* *initial_pose* (geometry_msgs/Pose)
    
    Set the initial pose of the robot if no last pose is available
