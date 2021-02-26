#include <gtest/gtest.h>

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

#include "mars_topology_common/utility/RegistrationGuard.hpp"

TEST(TopologyEdgeTests, initialization)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Local Edge");

  mars::routing::common::topology::Edge *lEdge;

  EXPECT_NO_THROW(lEdge = new mars::routing::common::topology::Edge(lEdgeId));
  EXPECT_NO_THROW(lEdge->getId());
  EXPECT_EQ(lEdgeId, lEdge->getId());
}

TEST(TopologyEdgeTests, direct_location)
{
  mars::common::Id lEdgeId("01000000000000000000000000000000", "Local Edge");
  mars::routing::common::topology::Edge lEdge(lEdgeId);

  Eigen::Vector3d lEdgeLocation(1.3337, 2.5, -3.9);
  EXPECT_NO_THROW(lEdge.setLocation(lEdgeLocation));

  boost::optional<Eigen::Vector3d &> lOptionalEdgeLocation = lEdge.getLocation();
  EXPECT_TRUE(lOptionalEdgeLocation != boost::none);
  EXPECT_EQ(lEdgeLocation, *lOptionalEdgeLocation);

  mars::common::Id lUnformattedEdgeId("01000000000000000000000000000000");
  mars::routing::common::topology::Edge lCachedEdge(lUnformattedEdgeId);

  lOptionalEdgeLocation = lCachedEdge.getLocation();
  EXPECT_TRUE(lOptionalEdgeLocation != boost::none);
  EXPECT_EQ(lEdgeLocation, *lOptionalEdgeLocation);
}

TEST(TopologyEdgeTests, service_location_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<Eigen::Vector3d &> lOptionalEdgeLocation = lEdge.getLocation();
  EXPECT_TRUE(lOptionalEdgeLocation != boost::none);
  EXPECT_FLOAT_EQ((*lOptionalEdgeLocation)[0], 10);
  EXPECT_FLOAT_EQ((*lOptionalEdgeLocation)[1], 0);
  EXPECT_FLOAT_EQ((*lOptionalEdgeLocation)[2], 0);
}

TEST(TopologyEdgeTests, service_location_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<Eigen::Vector3d &> lOptionalEdgeLocation = lEdge.getLocation();
  EXPECT_TRUE(lOptionalEdgeLocation == boost::none);
}

TEST(TopologyVertexTests, service_type_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  int lType = lEdge.getType();
  EXPECT_EQ(lType, 100);
}

TEST(TopologyVertexTests, service_type_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Vertex lEdge(lEdgeId);

  int lType = lEdge.getType();
  EXPECT_EQ(lType, 0);
}

TEST(TopologyEdgeTests, service_footprint_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<mars::common::geometry::Footprint &> lOptionalEdgeFootprint = lEdge.getFootprint();
  EXPECT_TRUE(lOptionalEdgeFootprint != boost::none);
}

TEST(TopologyEdgeTests, service_footprint_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<mars::common::geometry::Footprint &> lOptionalEdgeFootprint = lEdge.getFootprint();
  EXPECT_TRUE(lOptionalEdgeFootprint == boost::none);
}

TEST(TopologyEdgeTests, service_restrictions_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<mars::topology::common::TopologyEntityRestrictions &> lOptionalEdgeRestrictions =
      lEdge.getRestrictions();
  EXPECT_TRUE(lOptionalEdgeRestrictions != boost::none);

  EXPECT_DOUBLE_EQ(lOptionalEdgeRestrictions->getMaxLinearVelocity(), 10);
  EXPECT_DOUBLE_EQ(lOptionalEdgeRestrictions->getMaxAngularVelocity(), 1);
  EXPECT_DOUBLE_EQ(lOptionalEdgeRestrictions->getMaxLinearAcceleration(), 1);
  EXPECT_DOUBLE_EQ(lOptionalEdgeRestrictions->getMaxAngularAcceleration(), 0.5);
  EXPECT_DOUBLE_EQ(lOptionalEdgeRestrictions->getMaxHeight(), 2);
  EXPECT_DOUBLE_EQ(lOptionalEdgeRestrictions->getMaxTotalWeight(), 200);

  EXPECT_TRUE(lOptionalEdgeRestrictions->getForbiddenVehicleTypes().empty());
  EXPECT_TRUE(lOptionalEdgeRestrictions->getForbiddenHazardTypes().empty());
}

TEST(TopologyEdgeTests, service_restrictions_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<mars::topology::common::TopologyEntityRestrictions &> lOptionalEdgeRestrictions =
      lEdge.getRestrictions();
  EXPECT_TRUE(lOptionalEdgeRestrictions == boost::none);
}

TEST(TopologyEdgeTests, service_locked_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<bool> lIsLocked = lEdge.isLocked();
  EXPECT_TRUE(lIsLocked != boost::none);
  EXPECT_FALSE(*lIsLocked);
}

TEST(TopologyEdgeTests, service_locked_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<bool> lIsLocked = lEdge.isLocked();
  EXPECT_TRUE(lIsLocked == boost::none);
}

TEST(TopologyEdgeTests, is_traversable_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAPAllValid(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1,
                                                           0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPAllValid));

  mars::agent::physical::common::RobotAgentProperties lRAPLinearVelocityLimit(lAgentId, lVehicle, lFootprint, 10, 10, 0.1, 0.1,
                                                                      0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPLinearVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededLinearVelocityLimit(
      lAgentId, lVehicle, lFootprint, 20, 20, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPExceededLinearVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPLinearAccelerationLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 1,
                                                                          1, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPLinearAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededLinearAccelerationLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 2, 2, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPExceededLinearAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPAngularVelocityLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                       0.1, 1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPAngularVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededAngularVelocityLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 2, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPExceededAngularVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPAngularAccelerationLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.5, 0.5, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPAngularAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededAngularAccelerationLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPExceededAngularAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPHeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1,
                                                              0.1, 0.1, 2.0, 2.0, 10.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPHeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededMaxHeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                         0.1, 0.1, 0.1, 0.1, 1.0, 3.0, 10.0, 0.0);
  EXPECT_FALSE(lEdge.isTraversable(lRAPExceededMaxHeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededHeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                      0.1, 0.1, 0.1, 0.1, 2.5, 3.0, 10.0, 0.0);
  EXPECT_FALSE(lEdge.isTraversable(lRAPExceededHeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPWeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1,
                                                              0.1, 0.1, 1.0, 1.0, 199.0, 0.0);
  EXPECT_TRUE(lEdge.isTraversable(lRAPWeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededWeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                      0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 200.0001, 0.0);
  EXPECT_FALSE(lEdge.isTraversable(lRAPExceededWeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededWeightPayloadLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 100, 100.0001);
  EXPECT_FALSE(lEdge.isTraversable(lRAPExceededWeightPayloadLimit));
}

TEST(TopologyEdgeTests, is_traversable_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAP(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                   1.0, 1.0, 10.0, 0.0);

  EXPECT_FALSE(lEdge.isTraversable(lRAP));
}

TEST(TopologyEdgeTests, service_reservation_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  ros::Time lStartTime = ros::Time::now();
  boost::optional<std::vector<mars::common::TimeInterval>> lFreeTimeIntervals = lEdge.getFreeTimeIntervals(lStartTime);
  EXPECT_TRUE(lFreeTimeIntervals != boost::none);
  EXPECT_TRUE(lFreeTimeIntervals->size() == 1);
  EXPECT_TRUE(lFreeTimeIntervals->front().getStartTime() == lStartTime);
  EXPECT_TRUE(lFreeTimeIntervals->front().getEndTime() == mars::common::TimeInterval::INFINITE_END_TIME);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::Id lPathId;
  lPathId.initialize();

  mars::common::TimeInterval lReservationInterval =
      mars::common::TimeInterval(lFreeTimeIntervals->front().getStartTime(), lFreeTimeIntervals->front().getEndTime());
  EXPECT_TRUE(lEdge.addReservation(lAgentId, lPathId, lReservationInterval));
  lFreeTimeIntervals = lEdge.getFreeTimeIntervals(lStartTime);
  EXPECT_TRUE(lFreeTimeIntervals != boost::none);
  EXPECT_TRUE(lFreeTimeIntervals->empty());
  EXPECT_FALSE(lEdge.addReservation(lAgentId, lPathId, lReservationInterval));

  EXPECT_TRUE(lEdge.deleteReservation(lAgentId, lPathId));
  lFreeTimeIntervals = lEdge.getFreeTimeIntervals(lStartTime);
  EXPECT_TRUE(lFreeTimeIntervals != boost::none);
  EXPECT_TRUE(lFreeTimeIntervals->size() == 1);
}

TEST(TopologyEdgeTests, service_free_timeintervals_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  boost::optional<std::vector<mars::common::TimeInterval>> lFreeTimeIntervals =
      lEdge.getFreeTimeIntervals(ros::Time::now());
  EXPECT_TRUE(lFreeTimeIntervals == boost::none);
}

TEST(TopologyEdgeTests, service_allocation_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::Id lPathId;
  lPathId.initialize();
  EXPECT_FALSE(lEdge.deallocate(lAgentId));
  EXPECT_TRUE(lEdge.allocate(lAgentId, lPathId));
  EXPECT_FALSE(lEdge.deallocate(lAgentId));
}

TEST(TopologyEdgeTests, service_allocation_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::Id lPathId;
  lPathId.initialize();
  EXPECT_FALSE(lEdge.allocate(lAgentId, lPathId));
  EXPECT_FALSE(lEdge.deallocate(lAgentId));
}

TEST(TopologyEdgeTests, service_origin_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");
  mars::common::Id lOriginId("00000000000000000000000000000001", "Origin Vertex");
  mars::common::Id lTargetId("00000000000000000000000000000002", "Target Vertex");

  mars::routing::common::topology::Edge lEdge(lEdgeId);
  mars::routing::common::topology::Vertex lOrigin(lOriginId);
  mars::routing::common::topology::Vertex lTarget(lTargetId);

  boost::optional<mars::routing::common::topology::Vertex&> lRemoteOrigin = lEdge.getOrigin(lTarget);

  EXPECT_FALSE(lRemoteOrigin == boost::none);
  EXPECT_TRUE(*lRemoteOrigin == lOrigin);

  boost::optional<mars::routing::common::topology::Vertex&> lRemoteTarget = lEdge.getOrigin(lOrigin);
  EXPECT_TRUE(lRemoteTarget == boost::none);
}

TEST(TopologyEdgeTests, service_origin_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");
  mars::common::Id lTargetId("00000000000000000000000000000002", "Target Vertex");

  mars::routing::common::topology::Edge lEdge(lEdgeId);
  mars::routing::common::topology::Vertex lTarget(lTargetId);

  boost::optional<mars::routing::common::topology::Vertex&> lRemoteOrigin = lEdge.getOrigin(lTarget);

  EXPECT_TRUE(lRemoteOrigin == boost::none);
}

TEST(TopologyEdgeTests, service_target_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");
  mars::common::Id lOriginId("00000000000000000000000000000001", "Origin Vertex");
  mars::common::Id lTargetId("00000000000000000000000000000002", "Target Vertex");

  mars::routing::common::topology::Edge lEdge(lEdgeId);
  mars::routing::common::topology::Vertex lOrigin(lOriginId);
  mars::routing::common::topology::Vertex lTarget(lTargetId);

  boost::optional<mars::routing::common::topology::Vertex&> lRemoteTarget = lEdge.getTarget(lOrigin);

  EXPECT_FALSE(lRemoteTarget == boost::none);
  EXPECT_TRUE(*lRemoteTarget == lTarget);

  boost::optional<mars::routing::common::topology::Vertex&> lRemoteOrigin = lEdge.getTarget(lTarget);
  EXPECT_TRUE(lRemoteOrigin == boost::none);
}

TEST(TopologyEdgeTests, service_target_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");
  mars::common::Id lOriginId("00000000000000000000000000000001", "Origin Vertex");

  mars::routing::common::topology::Edge lEdge(lEdgeId);
  mars::routing::common::topology::Vertex lOrigin(lOriginId);

  boost::optional<mars::routing::common::topology::Vertex&> lRemoteTarget = lEdge.getTarget(lOrigin);

  EXPECT_TRUE(lRemoteTarget == boost::none);
}

TEST(TopologyEdgeTests, edge_traversable_targets_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");
  mars::common::Id lTargetId("00000000000000000000000000000002", "Target Vertex");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;
  std::vector<mars::routing::common::topology::Vertex> lTraversableTargets;

  mars::agent::physical::common::RobotAgentProperties lRAPAllValid(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1,
                                                           0.1, 1.0, 1.0, 10.0, 0.0);
  lTraversableTargets = lEdge.getTraversableTargets(lRAPAllValid);
  EXPECT_TRUE(lTraversableTargets.size() == 1);
  EXPECT_TRUE(lTraversableTargets.front().getId() == lTargetId);

  mars::agent::physical::common::RobotAgentProperties lRAPExceededWeightPayloadLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 100, 100.0001);
  lTraversableTargets = lEdge.getTraversableTargets(lRAPExceededWeightPayloadLimit);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
}

TEST(TopologyEdgeTests, edge_traversable_targets_unreachable)
{
  mars::common::Id lEdgeId("10000000000000000000000000000000", "Unreachable Edge");

  mars::routing::common::topology::Edge lEdge(lEdgeId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAP(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                   1.0, 1.0, 10.0, 0.0);

  EXPECT_TRUE(lEdge.getTraversableTargets(lRAP).empty());
}

TEST(TopologyEdgeTests, vertex_traversable_targets_reachable)
{
  mars::common::Id lEdgeId("00000000000000000000000000000003", "Reachable Edge");
  mars::common::Id lOriginId("00000000000000000000000000000001", "Origin Vertex");
  mars::common::Id lTargetId("00000000000000000000000000000002", "Target Vertex");

  mars::routing::common::topology::Edge lEdge(lEdgeId);
  mars::routing::common::topology::Vertex lOrigin(lOriginId);
  mars::routing::common::topology::Vertex lTarget(lTargetId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;
  std::vector<mars::routing::common::topology::Edge> lTraversableTargets;

  boost::optional<std::vector<mars::routing::common::topology::Edge>&> lOriginIngoingEdges = lOrigin.getIngoingEdges();
  EXPECT_FALSE(lOriginIngoingEdges == boost::none);
  EXPECT_TRUE(lOriginIngoingEdges->size() == 0);
  boost::optional<std::vector<mars::routing::common::topology::Edge>&> lTargetIngoingEdges = lTarget.getIngoingEdges();
  EXPECT_FALSE(lTargetIngoingEdges == boost::none);
  EXPECT_TRUE(lTargetIngoingEdges->size() == 1);
  EXPECT_TRUE(lTargetIngoingEdges->front() == lEdge);

  boost::optional<std::vector<mars::routing::common::topology::Edge>&> lOriginOutgoingEdges = lOrigin.getOutgoingEdges();
  EXPECT_FALSE(lOriginOutgoingEdges == boost::none);
  EXPECT_TRUE(lOriginOutgoingEdges->size() == 1);
  EXPECT_TRUE(lOriginOutgoingEdges->front() == lEdge);
  boost::optional<std::vector<mars::routing::common::topology::Edge>&> lTargetOutgoingEdges = lTarget.getOutgoingEdges();
  EXPECT_FALSE(lTargetOutgoingEdges == boost::none);
  EXPECT_TRUE(lTargetOutgoingEdges->size() == 0);

  // Check Traversability
  mars::agent::physical::common::RobotAgentProperties lRAPAllValid(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1,
                                                           0.1, 1.0, 1.0, 10.0, 0.0);
  lTraversableTargets = lOrigin.getTraversableIngoingEdges(lRAPAllValid);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
  lTraversableTargets = lOrigin.getTraversableOutgoingEdges(lRAPAllValid);
  EXPECT_TRUE(lTraversableTargets.size() == 1);
  EXPECT_TRUE(lTraversableTargets.front() == lEdge);
  lTraversableTargets = lOrigin.getTraversableTargets(lRAPAllValid);
  EXPECT_TRUE(lTraversableTargets.size() == 1);
  EXPECT_TRUE(lTraversableTargets.front() == lEdge);

  lTraversableTargets = lTarget.getTraversableIngoingEdges(lRAPAllValid);
  EXPECT_TRUE(lTraversableTargets.size() == 1);
  EXPECT_TRUE(lTraversableTargets.front() == lEdge);
  lTraversableTargets = lTarget.getTraversableOutgoingEdges(lRAPAllValid);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
  lTraversableTargets = lTarget.getTraversableTargets(lRAPAllValid);
  EXPECT_TRUE(lTraversableTargets.size() == 0);

  mars::agent::physical::common::RobotAgentProperties lRAPExceededWeightPayloadLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 100, 100.0001);
  lTraversableTargets = lOrigin.getTraversableIngoingEdges(lRAPExceededWeightPayloadLimit);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
  lTraversableTargets = lOrigin.getTraversableOutgoingEdges(lRAPExceededWeightPayloadLimit);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
  lTraversableTargets = lOrigin.getTraversableTargets(lRAPExceededWeightPayloadLimit);
  EXPECT_TRUE(lTraversableTargets.size() == 0);

  lTraversableTargets = lTarget.getTraversableIngoingEdges(lRAPExceededWeightPayloadLimit);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
  lTraversableTargets = lTarget.getTraversableOutgoingEdges(lRAPExceededWeightPayloadLimit);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
  lTraversableTargets = lTarget.getTraversableTargets(lRAPExceededWeightPayloadLimit);
  EXPECT_TRUE(lTraversableTargets.size() == 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TopologyEdgeTests");
  
  mars::topology::common::utility::RegistrationGuard( "00000000000000000000000000000003",
                                                      "00000000000000000000000000000001",
                                                      "00000000000000000000000000000002");

  return RUN_ALL_TESTS();
}