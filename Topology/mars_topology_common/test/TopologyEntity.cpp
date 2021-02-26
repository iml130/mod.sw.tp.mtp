#include "mars_topology_common/TopologyEntity.h"

#include "mars_topology_msgs/TopologyEntityType.h"
#include "mars_topology_srvs/GetTypeResponse.h"

#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>
#include <vector>

#include <ros/ros.h>

TEST(TopologyEntityTests, initialization)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  EXPECT_ANY_THROW(
      mars::topology::common::TopologyEntity lEdge(
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_ENTITY_TYPE_UNKNOWN,
          false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "edge_0",
          lRestrictions););

  mars::topology::common::TopologyEntity lEdge(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_EDGE, false, 0,
      0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "edge_0", lRestrictions);

  EXPECT_TRUE(lEdge.getLocks().empty());
  EXPECT_TRUE(lEdge.getReservations().empty());

  EXPECT_EQ(lEdge.getCoordinate().point.x, 0);
  EXPECT_EQ(lEdge.getCoordinate().point.y, 0);
  EXPECT_EQ(lEdge.getCoordinate().point.z, 0);

  EXPECT_EQ(lEdge.getFreeTimeSlots(ros::Time(0)).size(), 1);
}

TEST(TopologyEntityTests, addLock)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lEdge(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_EDGE, false, 0,
      0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "edge_0", lRestrictions);

  std::vector<mars::common::TimeInterval> freeTimeSlots;

  mars::common::Id lInitiatorId;
  lInitiatorId.initialize();

  // add a lock
  EXPECT_NO_THROW(lEdge.addLock(lInitiatorId, "For testing!", ros::Time(5),
                                ros::Duration(100), false));
  EXPECT_NO_THROW(lEdge.getFreeTimeSlots(ros::Time(0)));
  freeTimeSlots = lEdge.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(105), ros::Duration(-1)));

  // Add another lock in the timeinterval of the first
  EXPECT_NO_THROW(lEdge.addLock(lInitiatorId, "For testing!", ros::Time(10),
                                ros::Duration(20), false));
  EXPECT_NO_THROW(lEdge.getFreeTimeSlots(ros::Time(0)));
  freeTimeSlots = lEdge.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(105), ros::Duration(-1)));

  // add a lock in the future
  EXPECT_NO_THROW(lEdge.addLock(lInitiatorId, "For testing!", ros::Time(1000),
                                ros::Duration(20), false));
  EXPECT_NO_THROW(lEdge.getFreeTimeSlots(ros::Time(0)));
  freeTimeSlots = lEdge.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 3);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(105), ros::Duration(895)));
  EXPECT_EQ(freeTimeSlots[2],
            mars::common::TimeInterval(ros::Time(1020), ros::Duration(-1)));

  // add another lock in the future with infinite duration
  EXPECT_NO_THROW(lEdge.addLock(lInitiatorId, "For testing!", ros::Time(2000),
                                mars::common::TimeInterval::INFINITE_DURATION,
                                false));
  EXPECT_NO_THROW(lEdge.getFreeTimeSlots(ros::Time(0)));
  freeTimeSlots = lEdge.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 3);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(105), ros::Duration(895)));
  EXPECT_EQ(freeTimeSlots[2],
            mars::common::TimeInterval(ros::Time(1020), ros::Duration(980)));
}

TEST(TopologyEntityTests, addReservation)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntitysTests, addReservationTimeNow)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;
  mars::common::TimeInterval resInterval(ros::Time::now(), ros::Duration(10));

  agentOneId.initialize();
  pathOneId.initialize();

  // add a reservation
  EXPECT_NO_THROW(
      EXPECT_TRUE(lVertex.addReservation(agentOneId, pathOneId, resInterval)));

  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));

  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), resInterval.getStartTime()));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(resInterval.getEndTime(), ros::Duration(-1)));
}

TEST(TopologyEntityTests, addOverlappingReservation)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  mars::common::Id agentTwoId, pathTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  agentTwoId.initialize();
  pathTwoId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  // try to add a overlapping reservation
  EXPECT_NO_THROW(EXPECT_FALSE(lVertex.addReservation(
      agentTwoId, pathTwoId,
      mars::common::TimeInterval(ros::Time(6), ros::Duration(11)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, addContainedreservation)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  mars::common::Id agentTwoId, pathTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  agentTwoId.initialize();
  pathTwoId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  // try to add a reservation which is fully contained in our current
  // reservations
  EXPECT_NO_THROW(EXPECT_FALSE(lVertex.addReservation(
      agentTwoId, pathTwoId,
      mars::common::TimeInterval(ros::Time(6), ros::Duration(5)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, addOverlappingReservationSameAgentSamePath)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  // try to add a reservation which is fully contained in our current
  // reservations
  EXPECT_NO_THROW(EXPECT_FALSE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(6), ros::Duration(11)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, addOverlappingReservationSameAgentDifferentPath)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  mars::common::Id pathTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  pathTwoId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  // try to add a overlapping reservation
  EXPECT_NO_THROW(EXPECT_FALSE(lVertex.addReservation(
      agentOneId, pathTwoId,
      mars::common::TimeInterval(ros::Time(6), ros::Duration(11)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, addThreeNonOverlappingReservations)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  mars::common::Id agentTwoId, pathTwoId;
  mars::common::Id agentThreeId, pathThreeId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  agentTwoId.initialize();
  pathTwoId.initialize();

  agentThreeId.initialize();
  pathThreeId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));

  // add second reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentTwoId, pathTwoId,
      mars::common::TimeInterval(ros::Time(2), ros::Duration(1)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 3);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(2)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(3), ros::Duration(2)));
  EXPECT_EQ(freeTimeSlots[2],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));

  // add third reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentThreeId, pathThreeId,
      mars::common::TimeInterval(ros::Time(20), ros::Duration(2000)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 4);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(2)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(3), ros::Duration(2)));
  EXPECT_EQ(freeTimeSlots[2],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[3],
            mars::common::TimeInterval(ros::Time(2020), ros::Duration(-1)));
}

TEST(TopologyEntityTests, addAdjacentreservations)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  mars::common::Id agentTwoId, pathTwoId;
  mars::common::Id agentThreeId, pathThreeId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  agentTwoId.initialize();
  pathTwoId.initialize();

  agentThreeId.initialize();
  pathThreeId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(0), ros::Duration(10)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(10), ros::Duration(-1)));

  // add second reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentTwoId, pathTwoId,
      mars::common::TimeInterval(ros::Time(10), ros::Duration(25)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(35), ros::Duration(-1)));

  // add second reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentThreeId, pathThreeId,
      mars::common::TimeInterval(ros::Time(35), ros::Duration(1000)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(1035), ros::Duration(-1)));
}

TEST(TopologyEntityTests, addSurroundingReservation)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  mars::common::Id agentTwoId, pathTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  agentTwoId.initialize();
  pathTwoId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));

  // add second reservation
  EXPECT_NO_THROW(EXPECT_FALSE(lVertex.addReservation(
      agentTwoId, pathTwoId,
      mars::common::TimeInterval(ros::Time(3), ros::Duration(20)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, getFreeTime)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));
  // test free time windows from zero
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));

  // test free time windows from the reservation start
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(5));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));

  // test free time windows from the inside of reservation
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(8));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));

  // test free time windows after reservation
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(20));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, deleteReservation)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  EXPECT_NO_THROW(
      EXPECT_TRUE(lVertex.deleteReservation(agentOneId, pathOneId)));

  EXPECT_TRUE(lVertex.getReservations().empty());

  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(-1)));
}

TEST(TopologyEntityTests, deleteReservationDifferentAgentId)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId, agentTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();
  agentTwoId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  EXPECT_NO_THROW(
      EXPECT_FALSE(lVertex.deleteReservation(agentTwoId, pathOneId)));

  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, deleteReservationDifferentPathId)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId, pathTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();
  pathTwoId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  EXPECT_NO_THROW(
      EXPECT_FALSE(lVertex.deleteReservation(agentOneId, pathTwoId)));

  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}

TEST(TopologyEntityTests, deleteTwoReservations)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId, agentTwoId, pathTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();
  agentTwoId.initialize();
  pathTwoId.initialize();

  // add reservations
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));

  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentTwoId, pathTwoId,
      mars::common::TimeInterval(ros::Time(20), ros::Duration(10)))));

  // delete both
  EXPECT_NO_THROW(
      EXPECT_TRUE(lVertex.deleteReservation(agentOneId, pathOneId)));

  EXPECT_NO_THROW(
      EXPECT_TRUE(lVertex.deleteReservation(agentTwoId, pathTwoId)));

  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(-1)));
}

TEST(TopologyEntityTests, deleteNonExistReservation)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  EXPECT_NO_THROW(
      EXPECT_FALSE(lVertex.deleteReservation(agentOneId, pathOneId)));

  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(-1)));
}

TEST(TopologyEntityTests, freeTimeSlotsWithAllocation)
{
  mars::topology::common::TopologyEntityRestrictions lRestrictions;
  mars::topology::common::TopologyEntity lVertex(
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT,
      false, 0, 0, {-1, 1, 1, -1}, {1, 1, -1, -1}, "vertex_0", lRestrictions);

  mars::common::Id agentOneId, pathOneId;
  mars::common::Id agentTwoId, pathTwoId;
  std::vector<mars::common::TimeInterval> freeTimeSlots;

  agentOneId.initialize();
  pathOneId.initialize();

  agentTwoId.initialize();
  pathTwoId.initialize();

  // add a reservation
  EXPECT_NO_THROW(EXPECT_TRUE(lVertex.addReservation(
      agentOneId, pathOneId,
      mars::common::TimeInterval(ros::Time(5), ros::Duration(10)))));
  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 2);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(0), ros::Duration(5)));
  EXPECT_EQ(freeTimeSlots[1],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));

  //set allocation
  std::unordered_map<std::pair<mars::common::Id, mars::common::Id>,
                     mars::topology::common::TopologyEntityReservation*> reservations = lVertex.getReservations();

  lVertex.setAllocation(reservations.begin()->second);

  freeTimeSlots = lVertex.getFreeTimeSlots(ros::Time(0));
  EXPECT_EQ(freeTimeSlots.size(), 1);
  EXPECT_EQ(freeTimeSlots[0],
            mars::common::TimeInterval(ros::Time(15), ros::Duration(-1)));
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TopologyEntityTests");
  ros::NodeHandle lNH;
  return RUN_ALL_TESTS();
}
