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

// global agent ids
std::vector<mars::common::Id> agentIds = {
    mars::common::Id("AgentOne000000000000000000000000", "Agent_1"),
    mars::common::Id("AgentTwo000000000000000000000000", "Agent_2"),
    mars::common::Id("AgentThree0000000000000000000000", "Agent_3"),
    mars::common::Id("AgentFour00000000000000000000000", "Agent_4")};

/**
 * global path ids
 */
std::vector<mars::common::Id> pathIds = {
    mars::common::Id("PathOne0000000000000000000000000", "Path_1"),
    mars::common::Id("PathTwo0000000000000000000000000", "Path_2"),
    mars::common::Id("PathThree00000000000000000000000", "Path_3"),
    mars::common::Id("PathFour000000000000000000000000", "Path_4")};

/**
 * global topology entity ids
 */
std::vector<mars::common::Id> topologyEntityIds = {
    mars::common::Id("1950225e86e33e1394c61c4fa5bcafe4"), // allocateTwoAgents
    mars::common::Id("1950225e86e33e1394c61c4fa5bcafe5")  // allocateFourAgents
};

/**
 * condition for allocation test
 * Each index must be unique to one test
 */
std::vector<bool> testFinished = {
    false, // allocateTwoAgents
    false  // allocateFourAgents
};

// Action client callbacks

/**
 * @brief Constructs a result callback for the simple action client.
 * @param agent Index of the agent id in the global mapping
 * @param path Index of the path id in the global mapping
 * @param topologyEntity Index of the entity id in the global mapping
 * @param isLastAllocate Decides if the executing of this specific callback sets
 * the test finished flag
 * @param conditionIndex Index of the finished flag for this specific test in
 * the global mapping
 */
template <int agent, int path, int topologyEntity, bool isLastAllocate,
          int conditionIndex = -1>
void resultCallback(
    const actionlib::SimpleClientGoalState& state,
    const mars_topology_actions::AllocateEntityResultConstPtr& result)
{
  EXPECT_EQ(agentIds[agent], mars::common::Id(result->agent_id));
  EXPECT_EQ(pathIds[path], mars::common::Id(result->path_id));
  EXPECT_EQ(mars_common_msgs::Result::RESULT_SUCCESS, result->result.result);
  EXPECT_EQ(topologyEntityIds[topologyEntity],
            mars::common::Id(result->topology_entity_id));

  if (isLastAllocate && conditionIndex != -1)
  {
    // Set test to finished
    testFinished[conditionIndex] = true;
  }
}

/**
 * @brief Constructs a feedback callback for the simple action client.
 * @param agent Index of the agent id in the global mapping
 * @param path Index of the path id in the global mapping
 * @param topologyEntity Index of the entity id in the global mapping
 * @param initQueuePosition Expected queue position before any deallocating is
 * executed
 * @param waitCycles Number of feedbacks which should not reduce the expected
 * position in the queue. Needed due to the fact that an feedback is send on any
 * allocation while setting up the test scenario
 *
 */
template <int agent, int path, int topologyEntity, int initQueuePosition,
          int waitCycles>
void feedbackCallback(
    const mars_topology_actions::AllocateEntityFeedbackConstPtr& feedback)
{
  static int currentQueuePosition = initQueuePosition;
  static int currentWaitCycles = waitCycles;

  EXPECT_EQ(agentIds[agent], mars::common::Id(feedback->agent_id));
  EXPECT_EQ(pathIds[path], mars::common::Id(feedback->path_id));
  EXPECT_EQ(topologyEntityIds[topologyEntity],
            mars::common::Id(feedback->topology_entity_id));
  EXPECT_EQ(currentQueuePosition, feedback->position_in_queue);

  if (currentWaitCycles > 0)
  {
    currentWaitCycles--;
  }
  else
  {
    currentQueuePosition--;
  }
}

/*
 * @brief placeholder callback due to simpleActionClient sendGoal parameter
 * sequence
 */
void activeCallback() {}

TEST(ActionTests, allocateTwoAgents)
{
  // must be unique among all tests in this file
  const int testConditionIndex = 0;

  mars_topology_actions::AllocateEntityGoal allocateGoalOne, allocateGoalTwo;
  mars_topology_srvs::DeallocateEntity del;

  mars_topology_srvs::AddReservation res, res2;
  mars::common::TimeInterval resInterval(ros::Time(10), ros::Duration(5));
  mars::common::TimeInterval resInterval2(ros::Time(0), ros::Duration(5));

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentOne(
          "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe4/allocate/", true);

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentTwo(
          "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe4/allocate/", true);

  // Add a reservation
  res.request.reservation.agent_id = agentIds[0].toMsg();
  res.request.reservation.path_id = pathIds[0].toMsg();
  res.request.reservation.time_interval = resInterval.toMsg();

  // Add a reservation
  res2.request.reservation.agent_id = agentIds[1].toMsg();
  res2.request.reservation.path_id = pathIds[1].toMsg();
  res2.request.reservation.time_interval = resInterval2.toMsg();

  // add reservations
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe4/add_reservation",
      res));

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe4/add_reservation",
      res2));

  EXPECT_EQ(res.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(res2.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  // setup allocate goals
  allocateGoalOne.agent_id = agentIds[0].toMsg();
  allocateGoalOne.path_id = pathIds[0].toMsg();

  allocateGoalTwo.agent_id = agentIds[1].toMsg();
  allocateGoalTwo.path_id = pathIds[1].toMsg();

  // wait for actionserver to start
  clientAgentOne.waitForServer();
  clientAgentTwo.waitForServer();

  // send goals
  clientAgentOne.sendGoal(allocateGoalOne,
                          &resultCallback<0, 0, 0, true, testConditionIndex>,
                          &activeCallback, &feedbackCallback<0, 0, 0, 1, 1>);

  // feedback will not be called due to the reservation position (first)
  clientAgentTwo.sendGoal(allocateGoalTwo,
                          &resultCallback<1, 1, 0, false, testConditionIndex>);

  // simulate driving robot
  sleep(5);

  // deallocate
  del.request.agent_id = agentIds[1].toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe4/deallocate", del));

  EXPECT_EQ(del.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  ros::Rate rate(10);

  // wait for result of agent two
  while (!testFinished[testConditionIndex])
  {
    ros::spinOnce();
    rate.sleep();
  }
}

TEST(ActionTests, allocateFourAgents)
{
  // must be unique among all tests in this file
  const int testConditionIndex = 1;
  // length of the simulated agent driving before deallocation of topology entity in seconds
  const unsigned int simulatedDrivingLength = 3;

  mars_topology_actions::AllocateEntityGoal allocateGoalOne, allocateGoalTwo,
      allocateGoalThree, allocateGoalFour;
  mars_topology_srvs::DeallocateEntity deallocateAgentOne, deallocateAgentTwo,
      deallocateAgentThree, deallocateAgentFour;
  mars_topology_srvs::AddReservation resAgentOne, resAgentTwo, resAgentThree,
      resAgentFour;

  mars::common::TimeInterval resIntervalOne(ros::Time(0), ros::Duration(5));
  mars::common::TimeInterval resIntervalTwo(ros::Time(5), ros::Duration(5));
  mars::common::TimeInterval resIntervalThree(ros::Time(10), ros::Duration(5));
  mars::common::TimeInterval resIntervalFour(ros::Time(15), ros::Duration(5));

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentOne(
          "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/allocate/", true);

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentTwo(
          "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/allocate/", true);

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentThree(
          "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/allocate/", true);

  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>
      clientAgentFour(
          "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/allocate/", true);

  // setup reservations
  resAgentOne.request.reservation.agent_id = agentIds[0].toMsg();
  resAgentOne.request.reservation.path_id = pathIds[0].toMsg();
  resAgentOne.request.reservation.time_interval = resIntervalOne.toMsg();

  resAgentTwo.request.reservation.agent_id = agentIds[1].toMsg();
  resAgentTwo.request.reservation.path_id = pathIds[1].toMsg();
  resAgentTwo.request.reservation.time_interval = resIntervalTwo.toMsg();

  resAgentThree.request.reservation.agent_id = agentIds[2].toMsg();
  resAgentThree.request.reservation.path_id = pathIds[2].toMsg();
  resAgentThree.request.reservation.time_interval = resIntervalThree.toMsg();

  resAgentFour.request.reservation.agent_id = agentIds[3].toMsg();
  resAgentFour.request.reservation.path_id = pathIds[3].toMsg();
  resAgentFour.request.reservation.time_interval = resIntervalFour.toMsg();

  // setup allocate goals
  allocateGoalOne.agent_id = agentIds[0].toMsg();
  allocateGoalOne.path_id = pathIds[0].toMsg();

  allocateGoalTwo.agent_id = agentIds[1].toMsg();
  allocateGoalTwo.path_id = pathIds[1].toMsg();

  allocateGoalThree.agent_id = agentIds[2].toMsg();
  allocateGoalThree.path_id = pathIds[2].toMsg();

  allocateGoalFour.agent_id = agentIds[3].toMsg();
  allocateGoalFour.path_id = pathIds[3].toMsg();

  // setup deallocate srvs
  deallocateAgentOne.request.agent_id = agentIds[0].toMsg();

  deallocateAgentTwo.request.agent_id = agentIds[1].toMsg();

  deallocateAgentThree.request.agent_id = agentIds[2].toMsg();

  deallocateAgentFour.request.agent_id = agentIds[3].toMsg();

  // wait for actionserver to start
  clientAgentOne.waitForServer();
  clientAgentTwo.waitForServer();
  clientAgentThree.waitForServer();
  clientAgentFour.waitForServer();

  // send reservations
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/add_reservation",
      resAgentOne));

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/add_reservation",
      resAgentTwo));

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/add_reservation",
      resAgentThree));

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/add_reservation",
      resAgentFour));

  EXPECT_EQ(resAgentOne.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(resAgentTwo.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(resAgentThree.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);
  EXPECT_EQ(resAgentFour.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  // send allocations in reversed queue in order to trigger all feedbacks, by
  // simulating the worst case
  clientAgentFour.sendGoal(allocateGoalFour,
                           &resultCallback<3, 3, 1, true, testConditionIndex>,
                           &activeCallback, &feedbackCallback<3, 3, 1, 3, 3>);

  clientAgentThree.sendGoal(allocateGoalThree,
                            &resultCallback<2, 2, 1, false, testConditionIndex>,
                            &activeCallback, &feedbackCallback<2, 2, 1, 2, 2>);

  clientAgentTwo.sendGoal(allocateGoalTwo,
                          &resultCallback<1, 1, 1, false, testConditionIndex>,
                          &activeCallback, &feedbackCallback<1, 1, 1, 1, 1>);

  // feedback will not be called due to the reservation position (first)
  clientAgentOne.sendGoal(allocateGoalOne, &resultCallback<0, 0, 1, false>);


  //simulate driving agent
  sleep(simulatedDrivingLength);

  //deallocate topology entity
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/deallocate", deallocateAgentOne));

  EXPECT_EQ(deallocateAgentOne.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  //simulate driving agent
  sleep(simulatedDrivingLength);

  //deallocate topology entity
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/deallocate", deallocateAgentTwo));

  EXPECT_EQ(deallocateAgentTwo.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  //simulate driving agent
  sleep(simulatedDrivingLength);

  //deallocate topology entity
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/deallocate", deallocateAgentThree));

  EXPECT_EQ(deallocateAgentThree.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  //simulate driving agent
  sleep(simulatedDrivingLength);

  //deallocate topology entity
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe5/deallocate", deallocateAgentFour));

  EXPECT_EQ(deallocateAgentFour.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);


  // wait for getting the last result msg of the action
  ros::Rate rate(10);

  while (!testFinished[testConditionIndex])
  {
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestVertexActions");

  mars::topology::common::utility::RegistrationGuard(
      "1950225e86e33e1394c61c4fa5bcafe4", "1950225e86e33e1394c61c4fa5bcafe5");

  return RUN_ALL_TESTS();
}
