# **M**otion **T**ask **P**lanner - MTP

- [**M**otion **T**ask **P**lanner - MTP](#motion-task-planner---mtp)
  - [Background](#background)
  - [How it works](#how-it-works)
    - [Introduction](#introduction)
      - [Used Algorithm](#used-algorithm)
    - [The Routing Concept](#the-routing-concept)
    - [Agent descriptions](#agent-descriptions)
      - [Routing Agents](#routing-agents)
      - [Topology Agents](#topology-agents)
      - [Logical Agent](#logical-agent)
  - [Prerequisites](#prerequisites)
  - [Install](#install)
  - [Configuration](#configuration)
    - [Topology](#topology)
    - [Router](#router)
    - [Logical Agent(s)](#logical-agents)
  - [Interfaces](#interfaces)
    - [TP <-> RAN](#tp---ran)
    - [Interfaces produced by TP](#interfaces-produced-by-tp)
      - [MotionAssignment.msg](#motionassignmentmsg)
        - [ROS Message](#ros-message)
      - [CancelTask.msg](#canceltaskmsg)
        - [ROS Message](#ros-message-1)
    - [Interfaces consumed by TP](#interfaces-consumed-by-tp)
      - [Motion.msg](#motionmsg)
        - [ROS Message](#ros-message-2)
      - [AssignmentStatus.msg](#assignmentstatusmsg)
        - [ROS Message](#ros-message-3)
      - [RobotAgentDescription.msg](#robotagentdescriptionmsg)
        - [ROS Message](#ros-message-4)
    - [Used messages inside TP messages](#used-messages-inside-tp-messages)
      - [ID.msg](#idmsg)
        - [ROS Message](#ros-message-5)
      - [Sequence.msg](#sequencemsg)
        - [ROS Message](#ros-message-6)
      - [RobotAction.msg](#robotactionmsg)
        - [ROS Message](#ros-message-7)
      - [VehicleType.msg](#vehicletypemsg)
        - [ROS Message](#ros-message-8)
  - [License](#license)


## Background

[OPIL](https://opil-documentation.readthedocs.io/) is the Open Platform for Innovations in Logistcs. This platform is meant to enable the development of value added services for the logistics sector in small-scale Industry 4.0 contexts such as those of manufacturing SMEs. In fact, it provides an easy deployable suite of applications for rapid development of complete logistics solutions, including components for task scheduling, path planning, automatic factory layout generation and navigation.

This module is part of the TaskPlanner (TP). TP is one of the three OPIL Components and Functional blocks which this 3rd Layer (mod.sw.tp) is made of. Regarding the OPIL architecture, this node consists of two different sub-modules:

Firstly, the Task Supervisor (mod.sw.tp.ts) monitors the execution of the task dispatched to the agents (Robots). Secondly, the Motion Task Planning (mod.sw.tp.mtp) plans the motion tasks for the robot agents. Task Planner makes it possible for the different components to communicate with each other and be composed into full-fledged logistic system in a manufacturing environment.

The MOD.SW.TP Motion Task Planning module computes a motion task plan for the AGVs. This motion task provides a deadlock-free, optimal or near optimal path without loops and collision. Beyond this path computation the Motion Task Planning component handles the communication with the RAN. 

The MTP receives the start and destinations of the pending transport order (TO). It computes the best, shortest and/or fastest path for the and it handles the communication with the Robotic Agent Nodes (MOD.SW.RAN). Moreover, it is aware about the state, like current pose, current task, of the agents nodes.

## How it works

The main target of the Motion Task Planning is the calculation and coordination of robot paths. This ensures a safe coexistence between robots which operate together in the same environment. To certify this, the path planner will always try to avoid crossing paths or blockings. 

### Introduction

The coordination of robots in logistics environments has special requirements and constraints related to the used routing algorithms and data representations. In most cases, current routing algorithms do not follow the necessary requirements or are not able to integrate them due to inappropriate data representations. Special requirements for routing algorithms are  e.g. routing of multiple robots to the same destination, handling of blocking robots that may lead to a deadlock situation, creation of order sequencing, managing heterogeneous robot fleets. 

The handling of scalability and performance when the number of robots increases is also a crucial issue. Especially in the latter case, a decentralized approach can help to fulfill these requirements. As mentioned before, requirements to the routing appear on one side, on the other side the data representation of the logistics environment is a key factor. Special traffic regulations, like restricted driving directions (e.g. one way roads), or critical traffic situations, like narrow lanes, have to be mapped into a topological representation of the used routing algorithm. It is also necessary to depict vehicle positions and the positions of objects such as relocatable storage areas as a logistical environment which is in a state of constant changes, whereby all of these possible changes and requirements must be continuously updated in the data representation of the routing algorithm.

The used routing concept uses a topology which is based on a topological graph and a time window based routing.  

#### Used Algorithm

 The publication **C**ontext **A**ware **R**oute **P**lanning ***(CARP)*** approach of ***[ter Mors](https://link.springer.com/chapter/10.1007/978-3-642-16178-0_14 "Context-Aware Route Planning")*** presents a central approach for collision-free navigation of autonomous systems. 
 
 The algorithm is graph based and guarantees to calculate shortest paths from a start to a destination location, without colliding with any of the other robots, or ending up in a deadlock situation. For the path calculation and coordination time windows are used. Calculated routes can be defined as $\pi_n=\left(\left[r_1,\tau_1 \right], \dots,\left[r_n,\tau_n \right] \right), \tau_i=\left[t_i,t'_i\right)$  where $r_n$ is a node of the resource graph and $\tau_n$ is a time window that indicates that resource $r_n$ is available from time $t_i$ to $t'_i$, see subsequent figures. 
 
The basic idea of CARP can be defined as follows. Within routing, individual agents plan their routes successively one after the other. If an agent $n$ plans its route, the already planned $(n-1)$ routes are included in the current calculation. Every node of the resource graph has its own time windows that defines when the node is available. For planning conflict-free routes, overlapping time windows between the individual nodes of a route are required, in a way that $\tau\cap\tau'\neq\emptyset$. A calculation of the needed time windows takes place on the basis of real time values of the deployed vehicles. These values can be used to estimate the vehicle positions at a given time point. 

Based on this information the time windows of each node in the graph can be created. The algorithm also offers the possibility to integrate current events, such as delays, into the planning of future routes through a delay propagation approach.

![](./img/time_interval_routing_front.png)
![Visualization of planned CARP routes. Red circles depicts vertices of the topology, black arrows are the connecting edges of nodes. Blue and green lines on the floor (xyplane) depicts the planned path. The blue and green boxes along the axis depicts the planned path in the spacetime. This is the basis for CARP to plan conflict free path](./img/time_interval_routing_back.png)   

### The Routing Concept

 To establish a flexible and scalable path planning architecture, a new routing architecture will be presented, where an agent based approach is introduced. An overview of this agent model is shown in the subsequent figure. The chosen design provides the possibility of a simple decentralization as well as reduction of the complexity of the routing algorithm by dividing it into different agents. The presented approach consists of four different types of agents. Logical agents represent a software component which reacts to its environmental influences. In general, these agents have a higher level logic and serve as managers. Logical and **R**obot **A**gent **N**ode (RAN), (mod.iot.ran) have a one to one relation, thus a logical agent is permanently assigned to a specific RAN agent and viceversa. RAN agents act as a driver for the hardware and send driving commands or status information back to the logical agent. The responsibility for the routing is to calculate a path for the vehicle. This can be done in direct interaction with the topology. A route calculation is only possible within a direct coordination between the agents, whereby possible movements and solutions for critical situations are arranged. The developed routing concept acts mainly autonomously during the execution of an order. This includes the assignment of an order, the pick and place and subsequently the finish report. The interface to bring new orders to the system is the TaskSupervisor which can interact with all available logical agents.

 All mentioned agents will be described in detail in the section [Agent descriptions](#agent-descriptions).

### Agent descriptions

 This Section describes the developed decentralized agent based routing architecture in detail. A description of the responsibilities of the agents and the interaction among the agents is given. The main idea of this architecture relays on a distribution of responsibilities, knowledge and interactions among the agents. For this purpose, the agent offers services corresponding to its tasks. To get in contact with other specific agents, only the knowledge of the individual ID is necessary.

#### Routing Agents

The major task of the routing agent is to compute routes that are requested by the logical agents. For that reason, a routing service is offered to logical agents. This service allows to calculate real routes as well as test routes. The difference is time windows of test routes are not saved and do not serve as real routes. Information of a test route can be used during the negotiation process for new orders. While test routing, all necessary information, like other routes are taken into account. Hence a test route consists of the following information: travel distance, travel time and way points of the topology. Based on this information a better decision for an order can be made. 

The used routing algorithm inside the routing agent corresponds to the CARP algorithm which is presented in chapter [Used Algorithm](#used-algorithm). The basis of the route calculation is a topology, managed by the topology agent. After the launch of a routing agent, the agent has no knowledge about the topology. The topology is dynamically built during the first route requests. Therefore a route request consists of a start ID and destination ID. With this, all needed information can be queried from the topology agents. Queried topology information is cached by the routing agent. This ensures that basic information like connections, positions, restrictions, etc. are only requested once reducing network traffic and ensuring a maximum of dynamics. 

Free time windows and reservations of the topology are not cached and must be requested at any time required. After a route was successfully calculated by the routing agent, all calculated time slots are saved via a service call of the appropriate topology agent. Contrary to [Routing Concept](#The-Routing-Concept), the algorithm for the decentralized calculation of the routes has not yet been implemented in the current version of the router. Currently, the calculation of a route is only supported by one router at a time, but several router instances specialized for the calculation for certain vehicles can exist. This can be done e.g. by a token between the routers.

#### Topology Agents 

The topology agents depicts the current representation of the logistics environment as a graph based topology. This topology is offered to other agents, for example to the routing agent. In the case where a new route is calculated, the respective topology agents will be updated with reservation (time slots) of the calculated path. The reservation describes a time how long a vehicle or human will stay on a topology entity. In case of the topology, a topology entity can be represented by a vertex or an edge. One of the main tasks of a topology entity is the management of the reservations and calculation of free time slots for the routing through which new reservations can be calculated.  

For the time of a reservation, the topology entity is exclusively reserved for an specific agent. The calculation of reservations are part of the routing and are entered by the routing agent. In order to access a topology entity, it must be first reserved by a logical agent. Only the agent with the next reservation receives the surcharge. Other agents can already request the point for an allocation, but have to wait until it is released from the previously allocating agent. To know when a point is traveled or left, every topology entity has a footprint. The footprint describes an area that may not be navigated on if it has not been validly allocated, as well as an area that can be safely navigated on if it has been allocated. A footprint is described by a 2D polygon course. Every topology entity, vertex and edge, consists of such a footprint. 

Another important task of the topology is the management of important information like connections and restrictions. In case of a vertex, connections are edges which are connected with the vertex. In this case the vertex stores the ID of the connected edges, but has no further information. In case of an edge, the connecting vertex IDs (1 in case of an unidirectional edge, 2 in case of an bidirectional edge) are stored, also without further knowledge. It is the responsibility of the requesting agent to obtain further information from the adjacent agent. 

Restrictions of a topology entity can be for example a maximum velocity, special vehicle types, a maximum payload, etc. A topology entity can also be locked for a specific time interval. During this period a reservation and allocation of the topology entity is not possible for an agent. Locks are very helpful if something went wrong on the shop floor, or e.g. a vehicle has an technical problem. In terms of a delay, a delay propagation can be performed. In this case, the delayed logical agent informs the topology entity of the delay, causing it to postpone the reservation for the informing as well as all of the following logical agents. All following logical agents will be informed of the delay by the topology entity.

To reduce the number of individual running agents in the system, the topology run inside a topology container. A topology container is a wrapper for an set of vertex and edge agents. In terms of running agents individually they can run inside the container which is on the system view only one agent. A container can run $1$ to $n$ vertex or edge agents and $1$ to $m$ topology container can run simultaneously whereby an specific edge or vertex agent can only run inside one container at a time. From the architecture point of view all the individual vertex and edge agents are available as described above and can be contacted through the container. For the communication only an additional container id is needed which specifies where the vertex or edge agent runs. Test have shown that a huge number of agents on the same machine dramatically increase the amount of memory and CPU usage. In terms of representing a graph by agents the amount of individual agents is very high. For e.g. $20$x$20$ $4$-connected graph consists of 400 vertex and 1668 edge agents which results in more then 2000 agents only for the topology. Under the assumption that a vertex in a real environment has only a distance of 1 meter and 4 neighbors it is easy to see that the amount of topology agents can be very huge. 


#### Logical Agent

The logical agent is a high level manager for RAN, or in other words for a vehicle or human. Main task for the logical agent is e.g. the order management, which includes the participation of order actions, the route management and status updating of orders. 

New orders are assigned to the logical agents via order auctions. Here, the logical agent prepare bids for tendered contracts. A central instance selects the best logical agent for the respective order and assigns the order to it. In case a route is assigned to a logical agent, the agent retrieves the knowledge from the auction strategy and the order will be stored in a queue. 

Another main task of the logical agent is the route management. After a new task is picked from the queue, a path has to be calculated to the picking and placing location of the order. After a path has been received from the routing agent, the RAN agent can start traversing the path. Therefore the logical agents start with the allocation of topology entities which are a part of the calculated path and already reserved by the routing agent. 

After a topology entity is successfully allocated, the position can be send as a driving command to the RAN agent. The task of the logical agent is now to check whether the calculated time slots for this path segment are adhered to and the footprint of the path is not left by the RAN agent. In case the calculated time window cannot be kept, a delay propagation will be carried out by the logical agent. The challenge here is to predict how long the delay will be. However, since it can depend on a wide variety of influences, no prediction has been made. Influences can be e.g. blocked way, a person steps in front of a vehicle, the prediction of the time window was not precise, etc. Therefore a static time is currently propagated as delay. After a topology entity was traversed and the footprint of the topology entity is not longer occupied by the RAN agent footprint, the topology entity can be deallocated by the logical agent.

For handling errors, if the RAN agent e.g. is a vehicle, the logical agent continuously receives status information of the RAN agent. Beside information like position, finished orders, etc., errors are send to the logical agents. Some failures can be handled by the agent itself, others have to be handled by an operator. Protective field errors e.g. can be handled by the logical agent. If the error can not be handled by the logical agent, by waiting for a defined amount of time and propagating a delay, the error will be forwarded to an operator. 

## Prerequisites
* ROS1 Melodic or Noetic
* Further ROS dependencies: 
  * industrial-msgs
  * amcl
  * tf
  * map-server
  * tf2-geometry-msgs
  * angles
  * stage
  * tf2-eigen
  * eigen-conversions
  * geometry-msgs
  * laser-geometry
  * visualization-msgs
  * move-base-msgs
  * behaviortree-cpp-v3 (version 3.5.1)
* (Docker and docker-compose - in case you want to have an easy life ;))
* Understanding defining material flows based on [lotlan](https://lotlan.readthedocs.io/en/latest/)


## Install
You have to do the following steps:
* Install ROS Melodic or Noetic and create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
```
cd catkin_ws/src
git clone git@github.com:iml130/mod.sw.tp.mtp.git
```
* Optional: Install [FIWARE Orion Context Broker](https://fiware-orion.readthedocs.io/en/master/) via [Docker](https://hub.docker.com/r/fiware/orion/)

## Configuration

The MTP consists of several ROS nodes which needs to be startet. You need to start:

* mars_yellow_pages: A node which serves as a lookup service for topology container IDs.
* mars_topology_launcher: A node which launches the topology node(s) based on a topology file or a message received from mod.sw.sp.
* mars_routing_base: Is the routing node which calculates based on the topology conflict free path for the AGV.
* mars_agent_logical_agv: A node which serves as a manager for an AGV. The node receives transport orders from mod.sw.ts and process them in order. You have to start a mars_agent_logical_agv node for every AGV in the system.

The whole configuration file is listed below (mod_sw_tp.launch, preconfigured for one AGV:

```xml
<launch>

  <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0 0 0 0 0 0 1 world map" />

  <!-- ****** Yellow Pages ***** -->
  <include file="$(find mars_yellow_pages)/launch/mars_yellow_pages.launch" />

  <!--  ****** Topology *****  -->
  <include file="$(find mars_topology_launcher)/launch/mars_topology_launcher_generic.launch">
    <arg name="log_level" value="info" />
    <arg name="topo_file_type" value="opil_sp" />
    <arg name="mars_vertex_footprint_radius" value="0.95" />
    <arg name="topology_launch_mode" default="container"/>
  </include>

  <!-- ****** Router ***** -->
  <include file="$(find mars_routing_base)/launch/mars_routing_base.launch" />

  <!-- ****** Logical Agent (robot_0) ***** -->
  <include file="$(find mars_agent_logical_agv)/launch/mars_agent_logical_agv.launch">
    <arg name="robot_name" value="robot_opil_v2" />
    <arg name="physical_agent_id" value="00000000-0000-0000-0000-000000000001" />
    <arg name="physical_agent_description" value="robot_0" />
    <arg name="current_topology_entity_id" value="e53201ce-c3e3-53ed-b3df-daf27fcbb8e9" />
    <!-- Parking spot: P0 -->
    <arg name="parking_spot_entity_id" default="e53201ce-c3e3-53ed-b3df-daf27fcbb8e9" />
    <arg name="parking_spot_entity_type" default="10" />
    <arg name="parking_allowed" default="true" />

    <!-- ZFT hall rb1 setup -->
    <arg name="node_name" value="ran_00000000000000000000000000000001" />
    <arg name="physical_robot_namespace" value=""/>
  </include>

  <!-- ****** Firos ***** -->
  <node name="firos" pkg="firos" type="core.py"/>

  <node type="rviz" name="rviz" pkg="rviz" args="-d $(find mod_sw_tp)/rviz/config.rviz" />

</launch>
```

Following you find a more detailed description of the launch file and the parameter.

### Topology

Following you can see the configuration for the topology launcher module:

```xml
  <!-- ****** Yellow Pages ***** -->
  <include file="$(find mars_yellow_pages)/launch/mars_yellow_pages.launch" />
  
  <!--  ****** Topology *****  -->
  <include file="$(find mars_topology_launcher)/launch/mars_topology_launcher_generic.launch">
    <arg name="log_level" value="info" /> <!--log levels: debug, info, warn, error -->
    <arg name="topo_file_type" value="opil_sp" /> <!-- don't change this line -->
    <arg name="mars_vertex_footprint_radius" value="0.95" /> <!-- IMPORTANT: This value must be smaller (mars_vertex_footprint_radius < (cell_size / 2)) then the cell_size of SP!-->
    <arg name="topology_launch_mode" default="container"/>
  </include>
```

The log level for starting the topology can be set with the following command. Possible values are: **debug**, **info**, **warn**, **error**
```xml
<arg name="log_level" value="info" />
``` 

The **topo_file_type** parameter tells the program which kind of topology is expected. DON'T CHANGE THIS LINE!
```xml
<arg name="topo_file_type" value="opil_sp" /> 
```
The **mars_vertex_footprint_radius** describes the size of the bounding box which is created by the topology launcher around each vertex. Important: This value must be smaller (**mars_vertex_footprint_radius** < **(cell_size / 2)**) then the half of the **cell_size** of SP! Value is in meter. The next assumption is that the **mars_vertex_footprint_radius** is greater than the AGV turning radius defined in RANs robot description with the parameter **footprint_x** and **footprint_y**. It follows: **"AGV turning radius" < mars_vertex_footprint_radius < (cell_size / 2)**
```xml
<arg name="mars_vertex_footprint_radius" value="0.95" />
```
The **topology_launch_mode** sets the launch mode to **container**. This means that the topology is launched inside one container instead of individual agents. DON'T CHANGE THIS LINE!
```xml
<arg name="topology_launch_mode" default="container"/>
```

### Router
The Routing module which calculates the path for each robot can be started without any additional configuration. 
```xml
  <!-- ****** Router ***** -->
  <include file="$(find mars_routing_base)/launch/mars_routing_base.launch" />
```

### Logical Agent(s)
Each RAN of the system is represented by a logical agent. The logical agents manages the high level tasks, like receiving and managing transport orders from the TS. For version 3.X, one AGV is preconfigured. 

```xml
  <!-- ****** Logical Agent (robot_0) ***** -->
  <include file="$(find mars_agent_logical_agv)/launch/mars_agent_logical_agv.launch">
    <arg name="robot_name" value="robot_opil_v2" />
    <arg name="physical_agent_id" value="00000000-0000-0000-0000-000000000001" />
    <arg name="physical_agent_description" value="robot_0" />
    <arg name="current_topology_entity_id" value="e53201ce-c3e3-53ed-b3df-daf27fcbb8e9" />
    <!-- Parking spot: P0 -->
    <arg name="parking_spot_entity_id" default="e53201ce-c3e3-53ed-b3df-daf27fcbb8e9" />
    <arg name="parking_spot_entity_type" default="10" />
    <arg name="parking_allowed" default="true" />

    <!-- ZFT hall rb1 setup -->
    <arg name="node_name" value="ran_00000000000000000000000000000001" />
    <arg name="physical_robot_namespace" value=""/>
  </include>
```

The parameter **physical_robot_namespace** configures the namespace used by the ran. For e.g.: /opil/iot/ (In v3.X and above no namespace is configured)
```xml
<arg name="physical_robot_namespace" value=""/>
```

Name of the robot. Used to subscribe to the right robot topics. Layout of the topic structure: /physical_robot_namespace/robot_name/topics
```xml
<arg name="robot_name" value="robot_opil_v2" />
```

UUID of the physical agent. Id can be created randomly (UUID v4) or from a name (UUID v5). Important: The ID must be unique! For more information about UUID visit: https://en.wikipedia.org/wiki/Universally_unique_identifier
```xml
<arg name="physical_agent_id" value="00000000-0000-0000-0000-000000000001" />
```

Human readable name of the robot.
```xml
<arg name="physical_agent_description" value="robot_0" />
```

ID of the current node or edge on which the robot is located. Node end edge names are generated by SP. To translate a node oder edge name into a uuid, use UUID v5.
```xml
<arg name="current_topology_entity_id" value="0a8b9081-d84c-5660-909c-134d55bf4966" />
```

Name of the node. Topics and service are published as followed: /namespace/node_name/topic|service
```xml
<arg name="node_name" value="ran_00000000000000000000000000000001" />
```

ID of the node where the robot goes parking. A robot drives to this location if it doesn't receive a new order.
```xml
<arg name="parking_spot_entity_id" default="e53201ce-c3e3-53ed-b3df-daf27fcbb8e9" />
```

Type of the parking node. Currently not used.
```xml
<arg name="parking_spot_entity_type" default="10" />
```

Allows the AGV to go parking if it's idle.
```xml
    <arg name="parking_allowed" default="true" />
```


## Interfaces

Subsequently a detailed description of the exchanged entites is given. 

### TP <-> RAN

The following listing depicts the complete exchanged messages between RAN and TP. Subsequently all key value pairs will be explained in detail.

```json
{
    "id": "robot_opil_v2",
    "type": "ROBOT",
    "action_assignment": {
     ... },
    "cancel_order": {
     ... },
    "current_motion": {
     ... },
    "motion_assignment": {
     ... },
    "robot_description": {
     ... }
}
```

### Interfaces produced by TP

In the following all messages will be explained which are send by the TP. Messages might be based on some non-primitive types (e.g. mars_common_msgs/Id and others); These types are explained at the end of this document ([Used messages inside TP messages](#used-messages-inside-tp-messages)).

#### MotionAssignment.msg

The motion assignment tells the AGV the next destionation and under which circumstances it can moves to this position.

| Type                         | Variable         | Description                                                                                                                                                                                                       |
| ---------------------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id          | point_id         | ID of the next vertex / edge were the AGV should drive to.                                                                                                                                                        |
| mars_common_msgs/Id          | task_id          | ID of the task to which the MotionAssignment belongs.                                                                                                                                                             |
| mars_common_msgs/Id          | motion_id        | ID of the MotionAssignment. A new ID must be generated for each MotionAssignment.                                                                                                                                 |
| bool                         | is_waypoint      | TRUE if the point is a waypoint (intermediate point along the path), FALSE if it is a goal.                                                                                                                       |
| bool                         | use_orientation  | TRUE if the theta of the point has to be considered.                                                                                                                                                              |
| geometry_msgs/Twist          | max_velocity     | Maximum allowed velocity in the current segment. Segment is defined by the **motion_area**. (For more information about the message visit: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)            |
| geometry_msgs/Accel          | max_acceleration | Maximum allowed acceleration in the current segment. Segment is defined by the **motion_area**. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Accel.html) |
| geometry_msgs/PolygonStamped | motion_area      | Area in which the vehicle can move freely. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PolygonStamped.html)                                             |
| Sequence                     | sequence         | Sequence number of the current MotionAssignment.                                                                                                                                                                  |

##### ROS Message
```ini
Header header
mars_common_msgs/Id point_id
mars_common_msgs/Id task_id
mars_common_msgs/Id motion_id
geometry_msgs/Pose2D point
# TRUE if the point is a waypoint, FALSE if it is a goal
bool is_waypoint
# TRUE if the theta of the point has to be considered
bool use_orientation
geometry_msgs/Twist max_velocity
geometry_msgs/Accel max_acceleration
# defines the area in which the robot can move
geometry_msgs/PolygonStamped motion_area
# the position of the assignment in the sequence
Sequence sequence
```

#### CancelTask.msg

The cancel task message cancels a whole task for an AGV.

| Type                | Variable  | Description                                                                                                                 |
| ------------------- | --------- | --------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id | task_id   | ID of the task which should be canceled. If an Action- or MotionID is additionally given, only this part will be cancelled. |
| mars_common_msgs/Id | action_id | NOT supported at the moment!                                                                                                |
| mars_common_msgs/Id | motion_id | Not supported at the moment!                                                                                                |

##### ROS Message

```ini
# task ID instead of action id because the message deletes the whole task
# the task is a sequence of motions and actions
mars_common_msgs/Id task_id
mars_common_msgs/Id action_id
mars_common_msgs/Id motion_id
```

### Interfaces consumed by TP

#### Motion.msg

The motion message is a combination of the current position of the AGV in the global coordinate system provided by S&P and the current velocity. 

| Type                      | Variable         | Description                                                                                                                                          |
| ------------------------- | ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| geometry_msgs/PoseStamped | current_position | Current position of the AGV. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) |
| geometry_msgs/Twist       | current_velocity | Current velocity of the AGV. (For more information about the message visit: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)              |

##### ROS Message

```ini
geometry_msgs/PoseStamped current_position
geometry_msgs/Twist current_velocity
```

#### AssignmentStatus.msg

The assignment status gives you an overview which task is currently executed by the AGV and which was the last finished task.

| Type                         | Variable             | Description                                           |
| ---------------------------- | -------------------- | ----------------------------------------------------- |
| mars_common_msgs/Id          | current_task_id      | Id of the current task which is executed.             |
| mars_common_msgs/Id          | current_motion_id    | Id of the current MotionAssignment which is executed. |
| mars_common_msgs/Id          | current_action_id    | Id of the current ActionAssignment which is executed. |
| mars_common_msgs/Id          | last_finished_motion | Id of the last finished MotionAssignment.             |
| mars_common_msgs/Id          | last_finished_action | Id of the last finished ActionAssignment.             |
| geometry_msgs/PolygonStamped | footprint            | Current footprint of the AGV, including load.         |

##### ROS Message

```ini
mars_common_msgs/Id current_task_id
mars_common_msgs/Id current_motion_id
mars_common_msgs/Id current_action_id
mars_common_msgs/Id last_finished_motion
mars_common_msgs/Id last_finished_action
geometry_msgs/PolygonStamped footprint
```

#### RobotAgentDescription.msg

This message describes the AGVs footprint, kinematic and the capabilities like lifting operations. 

| Type                         | Variable          | Description                                                                                                                                                                                                                                        |
| ---------------------------- | ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id          | robot_id          | ID of the robot                                                                                                                                                                                                                                    |
| VehicleType                  | type              | Defines the type of the AGV                                                                                                                                                                                                                        |
| geometry_msgs/PolygonStamped | footprint         | The footprint is the contour of the mobile base. In ROS, it is a two dimensional array of the form [x0, y0],[x1, y1], ..., [xn, yn]]. The origin of the coordinates should be the center of the robot (center of rotation for differential drive). |
| float32                      | min_height        | Minimal height of the AGV in meter.                                                                                                                                                                                                                |
| float32                      | max_height        | Maximal height of the AGV in meter.                                                                                                                                                                                                                |
| float32                      | payload           | Maximum Payload which can be carried by the AGV in kilogram.                                                                                                                                                                                       |
| float32                      | max_pos_x_vel     | Maximum positive velocity in driving direction in m/s.                                                                                                                                                                                             |
| float32                      | max_neg_x_vel     | Maximum negative speed in reverse direction in m/s.                                                                                                                                                                                                |
| float32                      | max_pos_x_acc     | Maximum positive acceleration in driving direction in m/s².                                                                                                                                                                                        |
| float32                      | max_neg_x_acc     | Maximum negative acceleration in driving direction in m/s².                                                                                                                                                                                        |
| float32                      | max_pos_y_vel     | Maximum positive velocity in y direction for omnidirectional AGV in m/s.                                                                                                                                                                           |
| float32                      | max_neg_y_vel     | Maximum negative velocity in y direction for omnidirectional AGV in m/s.                                                                                                                                                                           |
| float32                      | max_pos_y_acc     | Maximum positive acceleration in y direction for omnidirectional AGV in m/s².                                                                                                                                                                      |
| float32                      | max_neg_y_acc     | Maximum negative acceleration in y direction for omnidirectional AGV in m/s².                                                                                                                                                                      |
| float32                      | max_pos_ang_v     | Maximum positive angular velocity in m/s.                                                                                                                                                                                                          |
| float32                      | max_neg_ang_v     | Maximum negative angular velocity in m/s.                                                                                                                                                                                                          |
| float32                      | max_pos_ang_a     | Maximum positive angular acceleration in m/s².                                                                                                                                                                                                     |
| float32                      | max_neg_ang_a     | Maximum negative angular acceleration in m/s².                                                                                                                                                                                                     |
| float32                      | velocity_cont     | ???                                                                                                                                                                                                                                                |
| float32                      | min_turning_r     | Turning radius in meter. For differential drives it is zero!                                                                                                                                                                                       |
| float32                      | batt_capacity     | Maximum capacity of the battery in Ah.                                                                                                                                                                                                             |
| float32                      | batt_max_volt     | Maximum voltage of the battery in V.                                                                                                                                                                                                               |
| float32                      | weight            | Weight of the AGV in kg.                                                                                                                                                                                                                           |
| string                       | vendor            | Vendor of the AGV.                                                                                                                                                                                                                                 |
| RobotAction []               | action_capability | A list of Actions which can be performed by the AGV.                                                                                                                                                                                               |

##### ROS Message

```ini
mars_common_msgs/Id robot_id
VehicleType type
geometry_msgs/PolygonStamped footprint
float32 min_height
float32 max_height
float32 payload
float32 max_pos_x_vel
float32 max_neg_x_vel
float32 max_pos_x_acc
float32 max_neg_x_acc
float32 max_pos_y_vel
float32 max_neg_y_vel
float32 max_pos_y_acc
float32 max_neg_y_acc
float32 max_pos_ang_vel
float32 max_neg_ang_vel
float32 max_pos_ang_acc
float32 max_neg_ang_acc
float32 velocity_control_sensitivity
float32 min_turning_radius
float32 batt_capacity
float32 batt_max_voltage
float32 weight
string vendor 
RobotAction[] action_capability

```

### Used messages inside TP messages

Used messages are not directly send to another participant in the system. These messages are only a part of other messages.

#### ID.msg
| Type      | Variable    | Description                                                                                                                                                                                                                                               |
| --------- | ----------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| uint8[16] | uuid        | Universally Unique Identifier (UUID). A UUID is a 128-bit number used to identify information in computer systems.  For generation UUIDs version 4 and 5 is used. For more information visit: https://en.wikipedia.org/wiki/Universally_unique_identifier |
| string    | description | Optional description of the ID. Description can be: Name, what is descriped by the id, etc.                                                                                                                                                               |
##### ROS Message
```ini
# Universally Unique Identifier (UUID)
# A UUID is a 128-bit number used to identify information in computer systems. 
# The term globally unique identifier (GUID) is also used. 
# More information: https://en.wikipedia.org/wiki/Universally_unique_identifier
uint8[16] uuid

# optional description of the id
# description can be:
#   - a name
#   - what is descriped by the id
#   - etc.
string description

```

#### Sequence.msg

| Type  | Variable        | Description                                                |
| ----- | --------------- | ---------------------------------------------------------- |
| int32 | sequence_number | Actual position in the sequence. The sequence starts at 1! |
| int32 | length          | Indicates how long the whole sequence will be.             |

##### ROS Message

```ini
# Message for defining a sequence.

# Actual position in the sequence. The sequence starts at 1! 
int32 sequence_number
# Overall lenght of the sequence
int32 length
```

#### RobotAction.msg

IMPORTANT: The robot action is not finally defined and not used in OPIL v3! 

| Type                     | Variable                          | Description                                                                       |
| ------------------------ | --------------------------------- | --------------------------------------------------------------------------------- |
| uint8                    | CATEGORY_UNDEFINED = 0            | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_NONE = 5                 | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_LOAD = 10                | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_MANUAL_LOAD_START = 11   | CONSTANT VARIABLE. Can be used inside the message for setting the accategorytion. |
| uint8                    | CATEGORY_MANUAL_LOAD_DONE = 12    | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_UNLOAD = 20              | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_MANUAL_UNLOAD_START = 21 | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_MANUAL_UNLOAD_DONE = 22  | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_START_CHARGING = 30      | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_STOP_CHARGING = 31       | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | category                          | Describes the category of the action. See constants!                              |
| uint8                    | action                            | Action which should be executed. Possible actions mus be described by the AGV!    |
| mars_common_msgs/Tuple[] | attributes                        | Additional attributes which are needed to execute the action.                     |
| string                   | description                       | Human readable description of the action.                                         |

##### ROS Message

```ini
# definition of all the possible actionsmars_agent_physical_robot_msgs
uint8 CATEGORY_UNDEFINED = 0
uint8 CATEGORY_NONE = 5
uint8 CATEGORY_LOAD = 10
uint8 CATEGORY_MANUAL_LOAD_START = 11
uint8 CATEGORY_MANUAL_LOAD_DONE = 12
uint8 CATEGORY_UNLOAD = 20
uint8 CATEGORY_MANUAL_UNLOAD_START = 21
uint8 CATEGORY_MANUAL_UNLOAD_DONE = 22
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
```

#### VehicleType.msg

IMPORTANT: The vehicle type is not finally defined and not used in OPIL v3! 

| Type  | Variable                        | Description                                                                     |
| ----- | ------------------------------- | ------------------------------------------------------------------------------- |
| uint8 | VEHICLE_TYPE_UNKNOWN=0          | CONSTANT VARIABLE. Can be used inside the message for setting the vehicle type. |
| uint8 | VEHICLE_TYPE_SUPPLY_VEHICLE=1   | CONSTANT VARIABLE. Can be used inside the message for setting the vehicle type. |
| uint8 | VEHICLE_TYPE_ASSEMBLY_VEHICLE=2 | CONSTANT VARIABLE. Can be used inside the message for setting the vehicle type. |
| uint8 | vehicle_type                    | Vehicle type of the AGV. Currently assembly and supply AGV are supported.       |

##### ROS Message

```ini
# supported vehicle_types
uint8 VEHICLE_TYPE_UNKNOWN=0
# TODO:add correct vehicle_types!
uint8 VEHICLE_TYPE_SUPPLY_VEHICLE=1
uint8 VEHICLE_TYPE_ASSEMBLY_VEHICLE=2

# type of the vehicle
uint8 vehicle_type
```

## License
[APACHE2](LICENSE) ©