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

TEST(Route2x1Tests, route)
{
  mars::common::Id lAgentId;
  mars::common::Id lPathId;
  lAgentId.initialize();
  lPathId.initialize();

  mars::common::Id lOriginId("00000000000000000000000000000010");
  mars::common::Id lDestinationId("00000000000000000000000000000001");
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
      lRoute(lOrigin, lDestination, lRAP, lOrientation, lStartTime);

  EXPECT_TRUE(lRoute.isValid());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Route2x1Tests");
  
  mars::topology::common::utility::RegistrationGuard( "00000000000000000000000000000010",
                                                      "00000000000000000000000000000001",
                                                      "00000000000000000000000000000002");
  
  return RUN_ALL_TESTS();
}