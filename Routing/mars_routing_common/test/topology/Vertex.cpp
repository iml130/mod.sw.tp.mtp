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

#include <gtest/gtest.h>

#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_common/topology/Edge.h"

#include "mars_topology_common/utility/RegistrationGuard.hpp"

TEST(TopologyVertexTests, initialization)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Local Vertex");

  mars::routing::common::topology::Vertex *lVertex;

  EXPECT_NO_THROW(lVertex = new mars::routing::common::topology::Vertex(lVertexId));
  EXPECT_NO_THROW(lVertex->getId());
  EXPECT_EQ(lVertexId, lVertex->getId());
}

TEST(TopologyVertexTests, direct_location)
{
  mars::common::Id lVertexId("01000000000000000000000000000000", "Local Vertex");
  mars::routing::common::topology::Vertex lVertex(lVertexId);

  Eigen::Vector3d lVertexLocation(1.3337, 2.5, -3.9);
  EXPECT_NO_THROW(lVertex.setLocation(lVertexLocation));

  boost::optional<Eigen::Vector3d &> lOptionalVertexLocation = lVertex.getLocation();
  EXPECT_TRUE(lOptionalVertexLocation != boost::none);
  EXPECT_EQ(lVertexLocation, *lOptionalVertexLocation);

  mars::common::Id lUnformattedVertexId("01000000000000000000000000000000");
  mars::routing::common::topology::Vertex lCachedVertex(lUnformattedVertexId);

  lOptionalVertexLocation = lCachedVertex.getLocation();
  EXPECT_TRUE(lOptionalVertexLocation != boost::none);
  EXPECT_EQ(lVertexLocation, *lOptionalVertexLocation);
}

TEST(TopologyVertexTests, service_location_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<Eigen::Vector3d &> lOptionalVertexLocation = lVertex.getLocation();
  EXPECT_TRUE(lOptionalVertexLocation != boost::none);
  EXPECT_FLOAT_EQ((*lOptionalVertexLocation)[0], 1.337);
  EXPECT_FLOAT_EQ((*lOptionalVertexLocation)[1], -1.337);
  EXPECT_FLOAT_EQ((*lOptionalVertexLocation)[2], 0);
}

TEST(TopologyVertexTests, service_location_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<Eigen::Vector3d &> lOptionalVertexLocation = lVertex.getLocation();
  EXPECT_TRUE(lOptionalVertexLocation == boost::none);
}

TEST(TopologyVertexTests, service_type_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  int lType = lVertex.getType();
  EXPECT_EQ(lType, 10);
}

TEST(TopologyVertexTests, service_type_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  int lType = lVertex.getType();
  EXPECT_EQ(lType, 0);
}

TEST(TopologyVertexTests, service_footprint_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<mars::common::geometry::Footprint &> lOptionalVertexFootprint = lVertex.getFootprint();
  EXPECT_TRUE(lOptionalVertexFootprint != boost::none);
}

TEST(TopologyVertexTests, service_footprint_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<mars::common::geometry::Footprint &> lOptionalVertexFootprint = lVertex.getFootprint();
  EXPECT_TRUE(lOptionalVertexFootprint == boost::none);
}

TEST(TopologyVertexTests, service_restrictions_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<mars::topology::common::TopologyEntityRestrictions &> lOptionalVertexRestrictions =
      lVertex.getRestrictions();
  EXPECT_TRUE(lOptionalVertexRestrictions != boost::none);

  EXPECT_DOUBLE_EQ(lOptionalVertexRestrictions->getMaxLinearVelocity(), 10);
  EXPECT_DOUBLE_EQ(lOptionalVertexRestrictions->getMaxAngularVelocity(), 1);
  EXPECT_DOUBLE_EQ(lOptionalVertexRestrictions->getMaxLinearAcceleration(), 1);
  EXPECT_DOUBLE_EQ(lOptionalVertexRestrictions->getMaxAngularAcceleration(), 0.5);
  EXPECT_DOUBLE_EQ(lOptionalVertexRestrictions->getMaxHeight(), 2);
  EXPECT_DOUBLE_EQ(lOptionalVertexRestrictions->getMaxTotalWeight(), 200);

  EXPECT_TRUE(lOptionalVertexRestrictions->getForbiddenVehicleTypes().empty());
  EXPECT_TRUE(lOptionalVertexRestrictions->getForbiddenHazardTypes().empty());
}

TEST(TopologyVertexTests, service_restrictions_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<mars::topology::common::TopologyEntityRestrictions &> lOptionalVertexRestrictions =
      lVertex.getRestrictions();
  EXPECT_TRUE(lOptionalVertexRestrictions == boost::none);
}

TEST(TopologyVertexTests, service_locked_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<bool> lIsLocked = lVertex.isLocked();
  EXPECT_TRUE(lIsLocked != boost::none);
  EXPECT_FALSE(*lIsLocked);
}

TEST(TopologyVertexTests, service_locked_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<bool> lIsLocked = lVertex.isLocked();
  EXPECT_TRUE(lIsLocked == boost::none);
}

TEST(TopologyVertexTests, is_traversable_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAPAllValid(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1,
                                                           0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPAllValid));

  mars::agent::physical::common::RobotAgentProperties lRAPLinearVelocityLimit(lAgentId, lVehicle, lFootprint, 10, 10, 0.1, 0.1,
                                                                      0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPLinearVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededLinearVelocityLimit(
      lAgentId, lVehicle, lFootprint, 20, 20, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPExceededLinearVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPLinearAccelerationLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 1,
                                                                          1, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPLinearAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededLinearAccelerationLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 2, 2, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPExceededLinearAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPAngularVelocityLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                       0.1, 1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPAngularVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededAngularVelocityLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 2, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPExceededAngularVelocityLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPAngularAccelerationLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.5, 0.5, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPAngularAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededAngularAccelerationLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 1.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPExceededAngularAccelerationLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPHeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1,
                                                              0.1, 0.1, 2.0, 2.0, 10.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPHeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededMaxHeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                         0.1, 0.1, 0.1, 0.1, 1.0, 3.0, 10.0, 0.0);
  EXPECT_FALSE(lVertex.isTraversable(lRAPExceededMaxHeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededHeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                      0.1, 0.1, 0.1, 0.1, 2.5, 3.0, 10.0, 0.0);
  EXPECT_FALSE(lVertex.isTraversable(lRAPExceededHeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPWeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1,
                                                              0.1, 0.1, 1.0, 1.0, 199.0, 0.0);
  EXPECT_TRUE(lVertex.isTraversable(lRAPWeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededWeightLimit(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                                      0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 200.0001, 0.0);
  EXPECT_FALSE(lVertex.isTraversable(lRAPExceededWeightLimit));

  mars::agent::physical::common::RobotAgentProperties lRAPExceededWeightPayloadLimit(
      lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 100, 100.0001);
  EXPECT_FALSE(lVertex.isTraversable(lRAPExceededWeightPayloadLimit));
}

TEST(TopologyVertexTests, is_traversable_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAP(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                   1.0, 1.0, 10.0, 0.0);

  EXPECT_FALSE(lVertex.isTraversable(lRAP));
}

TEST(TopologyVertexTests, service_reservation_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  ros::Time lStartTime = ros::Time::now();
  boost::optional<std::vector<mars::common::TimeInterval>> lFreeTimeIntervals =
      lVertex.getFreeTimeIntervals(lStartTime);
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
  EXPECT_TRUE(lVertex.addReservation(lAgentId, lPathId, lReservationInterval));
  lFreeTimeIntervals = lVertex.getFreeTimeIntervals(lStartTime);
  EXPECT_TRUE(lFreeTimeIntervals != boost::none);
  EXPECT_TRUE(lFreeTimeIntervals->empty());
  EXPECT_FALSE(lVertex.addReservation(lAgentId, lPathId, lReservationInterval));

  EXPECT_TRUE(lVertex.deleteReservation(lAgentId, lPathId));
  lFreeTimeIntervals = lVertex.getFreeTimeIntervals(lStartTime);
  EXPECT_TRUE(lFreeTimeIntervals != boost::none);
  EXPECT_TRUE(lFreeTimeIntervals->size() == 1);
}

TEST(TopologyVertexTests, service_free_timeintervals_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  boost::optional<std::vector<mars::common::TimeInterval>> lFreeTimeIntervals =
      lVertex.getFreeTimeIntervals(ros::Time::now());
  EXPECT_TRUE(lFreeTimeIntervals == boost::none);
}

TEST(TopologyVertexTests, service_allocation_reachable)
{
  mars::common::Id lVertexId("00000000000000000000000000000001", "Reachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::Id lPathId;
  lPathId.initialize();
  EXPECT_FALSE(lVertex.deallocate(lAgentId));
  EXPECT_TRUE(lVertex.allocate(lAgentId, lPathId));
  EXPECT_FALSE(lVertex.deallocate(lAgentId));
}

TEST(TopologyVertexTests, service_allocation_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::Id lPathId;
  lPathId.initialize();
  EXPECT_FALSE(lVertex.allocate(lAgentId, lPathId));
  EXPECT_FALSE(lVertex.deallocate(lAgentId));
}

TEST(TopologyVertexTests, vertex_targets_unreachable)
{
  mars::common::Id lVertexId("10000000000000000000000000000000", "Unreachable Vertex");

  mars::routing::common::topology::Vertex lVertex(lVertexId);

  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAP(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                   1.0, 1.0, 10.0, 0.0);

  EXPECT_TRUE(lVertex.getTraversableTargets(lRAP).empty());
  EXPECT_TRUE(lVertex.getIngoingEdges() == boost::none);
  EXPECT_TRUE(lVertex.getOutgoingEdges() == boost::none);
  EXPECT_TRUE(lVertex.getTraversableIngoingEdges(lRAP).empty());
  EXPECT_TRUE(lVertex.getTraversableOutgoingEdges(lRAP).empty());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TopologyVertexTests");

  mars::topology::common::utility::RegistrationGuard("00000000000000000000000000000001");

  return RUN_ALL_TESTS();
}