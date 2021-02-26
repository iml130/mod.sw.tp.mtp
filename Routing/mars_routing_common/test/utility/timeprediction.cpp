#include <gtest/gtest.h>

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/timeprediction.h"

#include "mars_topology_common/utility/RegistrationGuard.hpp"

TEST(UtilityTimepredictionTests, getMotionDuration)
{
  mars::common::Id lAgentId;
  lAgentId.initialize();
  mars::common::geometry::Footprint lFootprint;
  mars_agent_physical_robot_msgs::VehicleType lVehicle;
  lVehicle.vehicle_type = mars_agent_physical_robot_msgs::VehicleType::VEHICLE_TYPE_SUPPLY_VEHICLE;

  mars::agent::physical::common::RobotAgentProperties lRAP(lAgentId, lVehicle, lFootprint, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                   1.0, 1.0, 10.0, 0.0);

  ros::Duration lMotionDuration = mars::routing::common::utility::timeprediction<
      mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
      mars::routing::common::utility::AffineProfile>::getMotionDuration(123.45, lRAP);

  EXPECT_DOUBLE_EQ(lMotionDuration.toSec(), 246.9);

  lMotionDuration = mars::routing::common::utility::timeprediction<
      mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
      mars::routing::common::utility::AffineProfile>::getMotionDuration(0, lRAP);

  EXPECT_DOUBLE_EQ(lMotionDuration.toSec(), 0.0);

  lMotionDuration = mars::routing::common::utility::timeprediction<
      mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
      mars::routing::common::utility::AffineProfile>::getMotionDuration(-123.45, lRAP);

  EXPECT_DOUBLE_EQ(lMotionDuration.toSec(), 246.9);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "UtilityTimepredictionTests");

  mars::topology::common::utility::RegistrationGuard(
      "00000000000000000000000000000003", "00000000000000000000000000000001", "00000000000000000000000000000002");

  return RUN_ALL_TESTS();
}