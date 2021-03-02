//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

// include core libs
#include <gtest/gtest.h>
#include <ros/ros.h>

// include utility
#include <actionlib/client/simple_action_client.h>
#include <mars_common/Id.h>
#include <mars_common/TimeInterval.h>
#include <mars_topology_common/utility/RegistrationGuard.hpp>

// include actions
#include <mars_topology_actions/AllocateEntityAction.h>

//include services
#include <mars_topology_srvs/AddReservation.h>
#include <mars_topology_srvs/DeallocateEntity.h>
#include <mars_topology_srvs/DeleteReservation.h>
#include <mars_topology_srvs/GetCoordinate.h>
#include <mars_topology_srvs/GetFootprint.h>
#include <mars_topology_srvs/GetFreeTimeSlots.h>
#include <mars_topology_srvs/GetConnections.h>
#include <mars_topology_srvs/GetRestrictions.h>
#include <mars_topology_srvs/GetStatus.h>
#include <mars_topology_srvs/GetType.h>
#include <mars_topology_srvs/LockTopologyEntity.h>
#include <mars_topology_srvs/UnlockTopologyEntity.h>


//condition for allocation test
bool gotLastResult;

//Action client callbacks

void resultCallbackAgentOne(const actionlib::SimpleClientGoalState& state, const mars_topology_actions::AllocateEntityResultConstPtr& result)
{
 ROS_DEBUG("Result callback from agent two");

 EXPECT_EQ(mars::common::Id(result->agent_id),mars::common::Id("AgentOne000000000000000000000000","Agent_1"));
 EXPECT_EQ(mars::common::Id(result->path_id), mars::common::Id("PathOne0000000000000000000000000","Path_1"));
 EXPECT_EQ(result->result.result , mars_common_msgs::Result::RESULT_SUCCESS);
 EXPECT_EQ(mars::common::Id(result->topology_entity_id), mars::common::Id("00000000000000000000000000000007"));

 //set wait condition from the TEST
 gotLastResult = true;
}

void resultCallbackAgentTwo(const actionlib::SimpleClientGoalState& state, const mars_topology_actions::AllocateEntityResultConstPtr& result)
{
  EXPECT_EQ(mars::common::Id(result->agent_id),mars::common::Id("AgentTwo000000000000000000000000","Agent_2"));
  EXPECT_EQ(mars::common::Id(result->path_id), mars::common::Id("PathTwo0000000000000000000000000","Path_2"));
  EXPECT_EQ(result->result.result , mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(mars::common::Id(result->topology_entity_id), mars::common::Id("00000000000000000000000000000007"));
}

void feedbackCallbackAgentOne(const mars_topology_actions::AllocateEntityFeedbackConstPtr& feedback)
{
  EXPECT_EQ(mars::common::Id(feedback->agent_id),mars::common::Id("AgentOne000000000000000000000000","Agent_1"));
  EXPECT_EQ(mars::common::Id(feedback->path_id),mars::common::Id("PathOne0000000000000000000000000","Path_1"));
  EXPECT_EQ(feedback->position_in_queue, 1);
}

void feedbackCallbackAgentTwo(const mars_topology_actions::AllocateEntityFeedbackConstPtr& feedback)
{
  FAIL() << "Agent two should not get a feedback, but got one";
}

void activeCallback()
{}

void rosSpinFor(ros::Duration waitLength)
{
  ros::Time waitTime = ros::Time::now() + waitLength;

  while (ros::Time::now() < waitTime)
  {
    ros::spinOnce();
  }
}


TEST(ServicesTests, getCoordinate)
{
  mars_topology_srvs::GetCoordinate foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/get_coordinate", foo));

  EXPECT_EQ(foo.response.point.point.x, 4);
  EXPECT_EQ(foo.response.point.point.y, 5);
}

TEST(ServicesTests, getFootprint)
{
  mars_topology_srvs::GetFootprint foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/get_footprint", foo));

  EXPECT_EQ(foo.response.footprint.polygon.points[0].x, -2);
  EXPECT_EQ(foo.response.footprint.polygon.points[0].y, 2);

  EXPECT_EQ(foo.response.footprint.polygon.points[1].x, 2);
  EXPECT_EQ(foo.response.footprint.polygon.points[1].y, 2);

  EXPECT_EQ(foo.response.footprint.polygon.points[2].x, 2);
  EXPECT_EQ(foo.response.footprint.polygon.points[2].y, -2);

  EXPECT_EQ(foo.response.footprint.polygon.points[3].x, -2);
  EXPECT_EQ(foo.response.footprint.polygon.points[3].y, -2);
}

TEST(ServicesTests, getType)
{
  mars_topology_srvs::GetType foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/get_type", foo));

  EXPECT_EQ(foo.response.entity_type.entity_type,
            mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_EDGE);
}

TEST(ServicesTests, getStatus)
{
  mars_topology_srvs::AddReservation addRes;
  mars_topology_srvs::DeleteReservation delRes;

  mars_topology_srvs::GetStatus status;

  mars::common::Id agentId, pathId;
  mars::common::TimeInterval resInterval(ros::Time::now(), ros::Duration(10));

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/add_reservation",
      addRes));

  // Get Status
  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/get_status", status));

  EXPECT_TRUE(status.response.locks.empty());
  EXPECT_EQ(status.response.reservations.size(), 1);
  EXPECT_EQ(mars::common::Id(status.response.reservations[0].agent_id),
            agentId);
  EXPECT_EQ(mars::common::Id(status.response.reservations[0].path_id), pathId);
  EXPECT_EQ(
      mars::common::TimeInterval(status.response.reservations[0].time_interval),
      resInterval);

  // Delete Reservation
  delRes.request.agent_id = agentId.toMsg();
  delRes.request.path_id = pathId.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/delete_reservation",
      delRes));

  EXPECT_EQ(delRes.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  // Get Status
  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/get_status", status));

  EXPECT_TRUE(status.response.locks.empty());
  EXPECT_TRUE(status.response.reservations.empty());
}

TEST(ServicesTests, getRestrictions)
{
  mars_topology_srvs::GetRestrictions foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/get_restrictions", foo));

  EXPECT_EQ(foo.response.max_angular_velocity, 1);
  EXPECT_EQ(foo.response.max_angular_acceleration, 0.5);
  EXPECT_EQ(foo.response.max_height, 2);
  EXPECT_EQ(foo.response.max_linear_acceleration, 1);
  EXPECT_EQ(foo.response.max_linear_velocity, 10);
  EXPECT_EQ(foo.response.max_total_weight, 200);

  EXPECT_TRUE(foo.response.forbidden_vehicle_types.empty());
  EXPECT_TRUE(foo.response.forbidden_hazard_types.empty());
}

TEST(ServicesTests, addOneReservation)
{
  mars_topology_srvs::AddReservation addRes;
  mars::common::TimeInterval resInterval(ros::Time::now(), ros::Duration(10));
  mars::common::Id agentId, pathId;

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000004/add_reservation",
      addRes));

  EXPECT_EQ(addRes.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);
}

TEST(ServicesTests, freeTimeSlotOneReservation)
{
  mars_topology_srvs::AddReservation addRes;
  mars::common::TimeInterval resInterval(ros::Time::now(), ros::Duration(10));
  mars::common::Id agentId, pathId;
  mars_topology_srvs::GetFreeTimeSlots freeSlots;

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000005/add_reservation",
      addRes));

  EXPECT_EQ(addRes.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  // Check free time windows
  freeSlots.request.start_time = ros::Time(0);

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000005/get_free_time_slots",
      freeSlots));

  EXPECT_EQ(freeSlots.response.time_interval.size(), 2);

  mars::common::TimeInterval freeSlotOne(freeSlots.response.time_interval[0]);
  mars::common::TimeInterval freeSlotTwo(freeSlots.response.time_interval[1]);

  EXPECT_EQ(freeSlotOne, mars::common::TimeInterval(
                             ros::Time(0), resInterval.getStartTime()));
  EXPECT_EQ(freeSlotTwo, mars::common::TimeInterval(
                             resInterval.getEndTime(),
                             mars::common::TimeInterval::INFINITE_DURATION));
}

TEST(ServicesTests, deleteReservation)
{
  mars_topology_srvs::AddReservation addRes;
  mars_topology_srvs::DeleteReservation delRes;
  mars_topology_srvs::GetFreeTimeSlots freeSlots;

  mars::common::TimeInterval resInterval(ros::Time(210), ros::Duration(10));
  mars::common::Id agentId, pathId;

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000006/add_reservation",
      addRes));

  // delete the previous reservation
  delRes.request.agent_id = agentId.toMsg();
  delRes.request.path_id = pathId.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000006/delete_reservation",
      delRes));
}

TEST(ServicesTests, getConnectionsBidirectional)
{
  mars_topology_srvs::GetConnections con;

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000003/get_connections", con));

  mars::common::Id originId("00000000000000000000000000000001");
   mars::common::Id destinationId("00000000000000000000000000000002");

  EXPECT_EQ(mars::common::Id(con.response.connections[0].origin_vertex_id), originId);
  EXPECT_EQ(mars::common::Id(con.response.connections[0].destination_vertex_id), destinationId);

  EXPECT_EQ(mars::common::Id(con.response.connections[1].origin_vertex_id), destinationId);
  EXPECT_EQ(mars::common::Id(con.response.connections[1].destination_vertex_id), originId);
}

TEST(ServicesTests, getConnectionsUnidirectional)
{
  mars_topology_srvs::GetConnections con;

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000004/get_connections", con));

  mars::common::Id originId("00000000000000000000000000000001");
   mars::common::Id destinationId("00000000000000000000000000000002");

  EXPECT_TRUE(con.response.connections.size() == 1);
  EXPECT_EQ(mars::common::Id(con.response.connections[0].origin_vertex_id), originId);
  EXPECT_EQ(mars::common::Id(con.response.connections[0].destination_vertex_id), destinationId);
}

TEST(ServicesTests, allocateDeallocateEntity)
{
  mars_topology_actions::AllocateEntityGoal allocateGoalOne,
      allocateGoalTwo;
  mars_topology_srvs::AddReservation res, res2;
  mars_topology_srvs::DeallocateEntity del;
  mars::common::TimeInterval resInterval(ros::Time(10), ros::Duration(5));
  mars::common::TimeInterval resInterval2(ros::Time(0), ros::Duration(5));
  mars::common::Id agentId, pathId, agentIdTwo, pathIdTwo;

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentOne(
          "/topology/edge_00000000000000000000000000000007/allocate/", true);

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentTwo(
          "/topology/edge_00000000000000000000000000000007/allocate/", true);

  //init ids
  agentId.initialize("AgentOne000000000000000000000000","Agent_1");
  pathId.initialize("PathOne0000000000000000000000000", "Path_1");
  agentIdTwo.initialize("AgentTwo000000000000000000000000","Agent_2");
  pathIdTwo.initialize("PathTwo0000000000000000000000000", "Path_2");

  // Add a reservation
  res.request.reservation.agent_id = agentId.toMsg();
  res.request.reservation.path_id = pathId.toMsg();
  res.request.reservation.time_interval = resInterval.toMsg();

  // Add a reservation
  res2.request.reservation.agent_id = agentIdTwo.toMsg();
  res2.request.reservation.path_id = pathIdTwo.toMsg();
  res2.request.reservation.time_interval = resInterval2.toMsg();

  // add reservations
  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000007/add_reservation",
      res));

  EXPECT_TRUE(ros::service::call(
      "/topology/edge_00000000000000000000000000000007/add_reservation",
      res2));

  EXPECT_EQ(res.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(res2.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  //setup allocate goals
  allocateGoalOne.agent_id = agentId.toMsg();
  allocateGoalOne.path_id = pathId.toMsg();

  allocateGoalTwo.agent_id = agentIdTwo.toMsg();
  allocateGoalTwo.path_id = pathIdTwo.toMsg();

  //wait for actionserver to start
  clientAgentOne.waitForServer();
  clientAgentTwo.waitForServer();

  //set wait contion (global variable)
  gotLastResult = false;

  //send goals
  clientAgentOne.sendGoal(allocateGoalOne, &resultCallbackAgentOne, &activeCallback, &feedbackCallbackAgentOne);
  clientAgentTwo.sendGoal(allocateGoalTwo, &resultCallbackAgentTwo, &activeCallback, &feedbackCallbackAgentTwo);

  //simulate driving robot
  sleep(5);

  //deallocate
  del.request.agent_id = agentIdTwo.toMsg();

  EXPECT_TRUE(ros::service::call("/topology/edge_00000000000000000000000000000007/deallocate", del));

  EXPECT_EQ(del.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);

  //wait for result of agent two
  while (!gotLastResult)
  {
    rosSpinFor(ros::Duration(1));
  }
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestEdgeServices");

  mars::topology::common::utility::RegistrationGuard(
      "00000000000000000000000000000003", "00000000000000000000000000000004",
      "00000000000000000000000000000005", "00000000000000000000000000000006", "00000000000000000000000000000007");

  return RUN_ALL_TESTS();
}
