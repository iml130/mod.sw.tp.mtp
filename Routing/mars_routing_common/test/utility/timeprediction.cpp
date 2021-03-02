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