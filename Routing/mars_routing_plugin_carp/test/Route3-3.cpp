#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <vector>

#include <mars_routing_srvs/GetRoute.h>

#include "mars_common/Id.h"
#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_common/utility/AffineProfile.h"

#include "mars_routing_plugin_carp/Route.h"

#include "mars_topology_common/utility/RegistrationGuard.hpp"

#include <ros/ros.h>

TEST(Route33Tests, VertexToVertex)
{
  mars::common::Id lAgentId;
  mars::common::Id lPathId;
  lAgentId.initialize();
  lPathId.initialize();

  mars::common::Id lOriginId("00000000000000000000000000000010");
  mars::common::Id lDestinationId("00000000000000000000000000000005");
  mars::routing::common::topology::Vertex lOrigin(lOriginId);
  mars::routing::common::topology::Vertex lDestination(lDestinationId);

  ros::Time lStartTime = ros::Time::now();
  mars::common::geometry::Footprint lFootprint;

  double lOrientation = 0;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAP(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                   0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);

  mars::routing::plugin::carp::Route<
      mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
      mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>
      lRoute1(lOrigin, lDestination, lRAP, lOrientation, lStartTime);

  mars::routing::plugin::carp::Route<
      mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
      mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>
      lRoute2(lOrigin, lDestination, lRAP, lOrientation, lStartTime);

  EXPECT_TRUE(lRoute1.isValid());
  EXPECT_TRUE(lRoute2.isValid());
}

TEST(Route33Tests, EdgeToVertex)
{
  mars::common::Id lAgentId;
  mars::common::Id lPathId;
  lAgentId.initialize();
  lPathId.initialize();

  mars::common::Id lOriginId("00000000000000000000000000000008");
  mars::common::Id lDestinationId("00000000000000000000000000000002");
  mars::routing::common::topology::Edge lOrigin(lOriginId);
  mars::routing::common::topology::Vertex lDestination(lDestinationId);

  ros::Time lStartTime = ros::Time::now();
  mars::common::geometry::Footprint lFootprint;

  double lOrientation = 0;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAP(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1,
                                                   0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 0.0);

  mars::routing::plugin::carp::Route<
      mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
      mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>
      lRoute1(lOrigin, lDestination, lRAP, lOrientation, lStartTime);

  mars::routing::plugin::carp::Route<
      mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
      mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>
      lRoute2(lOrigin, lDestination, lRAP, lOrientation, lStartTime);

  EXPECT_TRUE(lRoute1.isValid());
  EXPECT_TRUE(lRoute2.isValid());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Route33Tests");
  
  mars::topology::common::utility::RegistrationGuard( "00000000000000000000000000000010",
                                                      "00000000000000000000000000000001",
                                                      "00000000000000000000000000000002",
                                                      "00000000000000000000000000000003",
                                                      "00000000000000000000000000000004",
                                                      "00000000000000000000000000000005",
                                                      "00000000000000000000000000000006",
                                                      "00000000000000000000000000000007",
                                                      "00000000000000000000000000000008",
                                                      "00000000000000000000000000000009",
                                                      "0000000000000000000000000000000a");

  return RUN_ALL_TESTS();
}