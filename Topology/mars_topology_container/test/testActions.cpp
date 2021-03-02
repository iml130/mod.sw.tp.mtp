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
#include <unordered_map>

// include actions
#include <mars_topology_actions/AllocateEntityAction.h>

// include utility
#include <actionlib/client/simple_action_client.h>
#include <mars_common/Id.h>
#include <mars_common/TimeInterval.h>
#include <mars_topology_common/utility/RegistrationGuard.hpp>

// include services
#include <mars_topology_srvs/AddReservation.h>
#include <mars_topology_srvs/DeallocateEntity.h>

#include <mars_common/Logger.h>
#include <thread>

// condition for terminate the test
bool gotLastResult;

// positions in queues for agents 2, 3 and 4

int positionQueue2 = 1;
int positionQueue3 = 2;
int positionQueue4 = 3;

// cycels before deallocating agent 1

int cyclesAgent2 = 1;
int cyclesAgent3 = 2;
int cyclesAgent4 = 3;

// Action client callbacks
void resultCallbackAgentFour(const actionlib::SimpleClientGoalState& state,
                             const mars_topology_actions::AllocateEntityResultConstPtr& result)
{
  ROS_DEBUG("00000000000000");

  EXPECT_EQ(mars::common::Id(result->agent_id),
            mars::common::Id("AgentFour00000000000000000000000", "Agent_4"));
  EXPECT_EQ(mars::common::Id(result->path_id),
            mars::common::Id("PathFour000000000000000000000000", "Path_4"));
  EXPECT_EQ(result->result.result, mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(mars::common::Id(result->topology_entity_id),
            mars::common::Id("00000000000000000000000000000002"));

  // set wait condition from the TEST
  gotLastResult = true;
}

void resultCallbackAgentThree(const actionlib::SimpleClientGoalState& state,
                              const mars_topology_actions::AllocateEntityResultConstPtr& result)
{
  ROS_DEBUG("00000000000000");

  EXPECT_EQ(mars::common::Id(result->agent_id),
            mars::common::Id("AgentThree0000000000000000000000", "Agent_3"));
  EXPECT_EQ(mars::common::Id(result->path_id),
            mars::common::Id("PathThree00000000000000000000000", "Path_3"));
  EXPECT_EQ(result->result.result, mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(mars::common::Id(result->topology_entity_id),
            mars::common::Id("00000000000000000000000000000002"));
}

void resultCallbackAgentTwo(const actionlib::SimpleClientGoalState& state,
                            const mars_topology_actions::AllocateEntityResultConstPtr& result)
{
  ROS_DEBUG("00000000000000");

  EXPECT_EQ(mars::common::Id(result->agent_id),
            mars::common::Id("AgentTwo000000000000000000000000", "Agent_2"));
  EXPECT_EQ(mars::common::Id(result->path_id),
            mars::common::Id("PathTwo0000000000000000000000000", "Path_2"));
  EXPECT_EQ(result->result.result, mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(mars::common::Id(result->topology_entity_id),
            mars::common::Id("00000000000000000000000000000002"));
}

void resultCallbackAgentOne(const actionlib::SimpleClientGoalState& state,
                            const mars_topology_actions::AllocateEntityResultConstPtr& result)
{
  ROS_DEBUG("00000000000000");

  EXPECT_EQ(mars::common::Id(result->agent_id),
            mars::common::Id("AgentOne000000000000000000000000", "Agent_1"));
  EXPECT_EQ(mars::common::Id(result->path_id),
            mars::common::Id("PathOne0000000000000000000000000", "Path_1"));
  EXPECT_EQ(result->result.result, mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(mars::common::Id(result->topology_entity_id),
            mars::common::Id("00000000000000000000000000000002"));
}

void feedbackCallbackAgentOne(const mars_topology_actions::AllocateEntityFeedbackConstPtr& feedback)
{
  FAIL() << "Agent one should not get a feedback, but got one";
}

void feedbackCallbackAgentTwo(const mars_topology_actions::AllocateEntityFeedbackConstPtr& feedback)
{
  ROS_ERROR_STREAM("Agent 2 Position in queue is " << positionQueue2 << std::endl);

  EXPECT_EQ(mars::common::Id(feedback->agent_id),
            mars::common::Id("AgentTwo000000000000000000000000", "Agent_2"));
  EXPECT_EQ(mars::common::Id(feedback->path_id),
            mars::common::Id("PathTwo0000000000000000000000000", "Path_2"));
  EXPECT_EQ(feedback->position_in_queue, positionQueue2);

  if (cyclesAgent2 > 0)
    cyclesAgent2--;
  else
    positionQueue2--;
}

void feedbackCallbackAgentThree(
    const mars_topology_actions::AllocateEntityFeedbackConstPtr& feedback)
{
  ROS_ERROR_STREAM("Agent 3 Position in queue is " << positionQueue3 << std::endl);

  EXPECT_EQ(mars::common::Id(feedback->agent_id),
            mars::common::Id("AgentThree0000000000000000000000", "Agent_3"));
  EXPECT_EQ(mars::common::Id(feedback->path_id),
            mars::common::Id("PathThree00000000000000000000000", "Path_3"));
  EXPECT_EQ(feedback->position_in_queue, positionQueue3);

  if (cyclesAgent3 > 0)
    cyclesAgent3--;
  else
    positionQueue3--;
}

void feedbackCallbackAgentFour(
    const mars_topology_actions::AllocateEntityFeedbackConstPtr& feedback)
{

  ROS_ERROR_STREAM("Agent 4 Position in queue is " << positionQueue4 << std::endl);

  EXPECT_EQ(mars::common::Id(feedback->agent_id),
            mars::common::Id("AgentFour00000000000000000000000", "Agent_4"));
  EXPECT_EQ(mars::common::Id(feedback->path_id),
            mars::common::Id("PathFour000000000000000000000000", "Path_4"));
  EXPECT_EQ(feedback->position_in_queue, positionQueue4);

  if (cyclesAgent4 > 0)
    cyclesAgent4--;
  else
    positionQueue4--;
}

void activeCallback() {}

// rosSpin for an amount of time
void rosSpinFor(ros::Duration(waitLength))
{
  ros::Time waitTime = ros::Time::now() + waitLength;
  while (ros::Time::now() < waitTime)
  {
    ros::spinOnce();
  }
}

TEST(ActionTests, fourAgentsAllocation)
{
  mars_topology_actions::AllocateEntityGoal allocateGoalOne, allocateGoalTwo, allocateGoalThree,
      allocateGoalFour;
  mars_topology_srvs::AddReservation res1, res2, res3, res4;
  mars_topology_srvs::DeallocateEntity del1, del2, del3;
  mars::common::TimeInterval resIntervalOne(ros::Time(0), ros::Duration(5));
  mars::common::TimeInterval resIntervalTwo(ros::Time(5), ros::Duration(5));
  mars::common::TimeInterval resIntervalThree(ros::Time(10), ros::Duration(5));
  mars::common::TimeInterval resIntervalFour(ros::Time(16), ros::Duration(5));
  mars::common::Id agentId1, pathId1, agentId2, pathId2, agentId3, pathId3, agentId4, pathId4;

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction> clientAgentOne(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/allocate/", true);

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction> clientAgentTwo(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/allocate/", true);
  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction> clientAgentThree(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/allocate/", true);

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction> clientAgentFour(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/allocate/", true);

  // init ids
  agentId1.initialize("AgentOne000000000000000000000000", "Agent_1");
  pathId1.initialize("PathOne0000000000000000000000000", "Path_1");
  agentId2.initialize("AgentTwo000000000000000000000000", "Agent_2");
  pathId2.initialize("PathTwo0000000000000000000000000", "Path_2");
  agentId3.initialize("AgentThree0000000000000000000000", "Agent_3");
  pathId3.initialize("PathThree00000000000000000000000", "Path_3");
  agentId4.initialize("AgentFour00000000000000000000000", "Agent_4");
  pathId4.initialize("PathFour000000000000000000000000", "Path_4");

  // Add first reservation
  res1.request.reservation.agent_id = agentId1.toMsg();
  res1.request.reservation.path_id = pathId1.toMsg();
  res1.request.reservation.time_interval = resIntervalOne.toMsg();
  res1.request.entity_id = mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  // Add second reservation
  res2.request.reservation.agent_id = agentId2.toMsg();
  res2.request.reservation.path_id = pathId2.toMsg();
  res2.request.reservation.time_interval = resIntervalTwo.toMsg();
  res2.request.entity_id = mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  // Add third reservation
  res3.request.reservation.agent_id = agentId3.toMsg();
  res3.request.reservation.path_id = pathId3.toMsg();
  res3.request.reservation.time_interval = resIntervalThree.toMsg();
  res3.request.entity_id = mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  // Add fourth reservation
  res4.request.reservation.agent_id = agentId4.toMsg();
  res4.request.reservation.path_id = pathId4.toMsg();
  res4.request.reservation.time_interval = resIntervalFour.toMsg();
  res4.request.entity_id = mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  // call services to add reservations
  EXPECT_TRUE(ros::service::call(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/add_reservation", res1));
  EXPECT_TRUE(ros::service::call(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/add_reservation", res2));
  EXPECT_TRUE(ros::service::call(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/add_reservation", res3));
  EXPECT_TRUE(ros::service::call(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/add_reservation", res4));

  EXPECT_EQ(res1.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(res2.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(res3.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(res4.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);

  // setup allocate goals
  allocateGoalOne.agent_id = agentId1.toMsg();
  allocateGoalOne.path_id = pathId1.toMsg();
  allocateGoalOne.topology_entity_id =
      mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();
  allocateGoalTwo.agent_id = agentId2.toMsg();
  allocateGoalTwo.path_id = pathId2.toMsg();
  allocateGoalTwo.topology_entity_id =
      mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();
  allocateGoalThree.agent_id = agentId3.toMsg();
  allocateGoalThree.path_id = pathId3.toMsg();
  allocateGoalThree.topology_entity_id =
      mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();
  allocateGoalFour.agent_id = agentId4.toMsg();
  allocateGoalFour.path_id = pathId4.toMsg();
  allocateGoalFour.topology_entity_id =
      mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  // wait for actionservicer to start
  clientAgentOne.waitForServer();
  clientAgentTwo.waitForServer();
  clientAgentThree.waitForServer();
  clientAgentFour.waitForServer();

  // set the globale variable on false, this marks that the test is still running
  gotLastResult = false;

  // send goals
  clientAgentFour.sendGoal(allocateGoalFour, &resultCallbackAgentFour, &activeCallback,
                           &feedbackCallbackAgentFour);
  
  clientAgentThree.sendGoal(allocateGoalThree, &resultCallbackAgentThree, &activeCallback,
                            &feedbackCallbackAgentThree);
  
  clientAgentTwo.sendGoal(allocateGoalTwo, &resultCallbackAgentTwo, &activeCallback,
                          &feedbackCallbackAgentTwo);
  
  clientAgentOne.sendGoal(allocateGoalOne, &resultCallbackAgentOne, &activeCallback,
                          &feedbackCallbackAgentOne);
  

  // simulate driving robot
  sleep(5);

  // deallocate agent 1
  del1.request.agent_id = agentId1.toMsg();
  del1.request.entity_id = mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  EXPECT_TRUE(
      ros::service::call("/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/deallocate", del1));
  EXPECT_EQ(del1.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);

  // simulate driving robot
  sleep(5);

  // deallocate agent 2
  del2.request.agent_id = agentId2.toMsg();
  del2.request.entity_id = mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  EXPECT_TRUE(
      ros::service::call("/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/deallocate", del2));
  EXPECT_EQ(del2.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);

  // simulate driving robot
  sleep(5);

  // deallocate agent 3
  del3.request.agent_id = agentId3.toMsg();
  del3.request.entity_id = mars::common::Id("00000000000000000000000000000002", "vertex02").toMsg();

  EXPECT_TRUE(
      ros::service::call("/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/deallocate", del3));
  EXPECT_EQ(del3.response.result.result, mars_common_msgs::Result::RESULT_SUCCESS);

  while (!gotLastResult)
  {
    rosSpinFor(ros::Duration(1));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestContainerActions");

  mars::topology::common::utility::RegistrationGuard("afd0b036625a3aa8b6399dc8c8fff0ff");
  ROS_ERROR_STREAM("test0000000000000000000000000" << std::endl);
  sleep(10);
  return RUN_ALL_TESTS();
}