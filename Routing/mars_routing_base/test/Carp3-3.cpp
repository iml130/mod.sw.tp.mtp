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
#include <eigen3/Eigen/Core>
#include <vector>

#include "mars_common/Id.h"
#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_core/Route.h"
#include "mars_routing_srvs/GetRoute.h"

#include "mars_topology_common/utility/RegistrationGuard.hpp"

#include <ros/ros.h>

TEST(Carp33Tests, simple_crossing)
{
  mars_routing_srvs::GetRoute lGetRouteService1;

  mars::common::Id lAgentId1;
  lAgentId1.initialize();

  mars::common::Id lAgentId2;
  lAgentId2.initialize();

  mars::common::Id lOriginId("00000000-0000-0000-0000-000000000010");
  mars::common::Id lDestinationId("00000000-0000-0000-0000-000000000005");

  EXPECT_NO_THROW(lGetRouteService1.request.route_request_type = lGetRouteService1.request.GET_ROUTE);
  EXPECT_NO_THROW(lGetRouteService1.request.start_time.data = ros::Time::now());
  EXPECT_NO_THROW(lGetRouteService1.request.origin.id = lOriginId.toMsg());
  EXPECT_NO_THROW(lGetRouteService1.request.origin.entity_type.entity_type =
                      lGetRouteService1.request.origin.entity_type.TOPOLOGY_VERTEX_TYPE_WAYPOINT);
  EXPECT_NO_THROW(lGetRouteService1.request.destination.id = lDestinationId.toMsg());
  EXPECT_NO_THROW(lGetRouteService1.request.destination.entity_type.entity_type =
                      lGetRouteService1.request.destination.entity_type.TOPOLOGY_VERTEX_TYPE_WAYPOINT);

  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_orientation = 0);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.robot_id = lAgentId1.toMsg());
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_angular_acceleration = 0.1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_angular_deceleration = 0.1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_angular_velocity = 0.1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_backward_velocity = 0.5);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_forward_velocity = 1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.min_height = 1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_height = 1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_linear_acceleration = 0.1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_linear_deceleration = 0.1);
  EXPECT_NO_THROW(lGetRouteService1.request.robot_agent_properties.max_payload = 0);

  mars_routing_srvs::GetRoute lGetRouteService2 = lGetRouteService1;

  EXPECT_TRUE(ros::service::call("/mars/routing/carp_11111111111111111111111111111111/GetRoute", lGetRouteService1));

  mars::routing::core::Route<mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
                             mars::routing::common::topology::Vertex>
      lRoute1(lGetRouteService1);

  EXPECT_TRUE(lRoute1.isValid());

  lOriginId = mars::common::Id("00000000-0000-0000-0000-000000000008");
  lDestinationId = mars::common::Id("00000000-0000-0000-0000-000000000002");

  EXPECT_NO_THROW(lGetRouteService2.request.origin.id = lOriginId.toMsg());
  EXPECT_NO_THROW(lGetRouteService2.request.origin.entity_type.entity_type =
                      lGetRouteService2.request.origin.entity_type.TOPOLOGY_EDGE_TYPE_EDGE);
  EXPECT_NO_THROW(lGetRouteService2.request.destination.id = lDestinationId.toMsg());
  EXPECT_NO_THROW(lGetRouteService2.request.destination.entity_type.entity_type =
                      lGetRouteService2.request.destination.entity_type.TOPOLOGY_VERTEX_TYPE_WAYPOINT);
  EXPECT_NO_THROW(lGetRouteService2.request.robot_agent_properties.robot_id = lAgentId2.toMsg());
  EXPECT_NO_THROW(lGetRouteService2.request.robot_agent_properties.max_forward_velocity = 1);

  EXPECT_TRUE(ros::service::call("/mars/routing/carp_11111111111111111111111111111111/GetRoute", lGetRouteService2));

  mars::routing::core::Route<mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
                             mars::routing::common::topology::Vertex>
      lRoute2(lGetRouteService2);

  EXPECT_TRUE(lRoute2.isValid());

  for (mars::routing::core::IterationStep& iStep : lRoute1)
  {
    EXPECT_TRUE(iStep.getTarget().deleteReservation(lAgentId1, lRoute1.getId()));
  }

  for (mars::routing::core::IterationStep& iStep : lRoute2)
  {
    EXPECT_TRUE(iStep.getTarget().deleteReservation(lAgentId2, lRoute2.getId()));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Carp33Tests");

  mars::topology::common::utility::RegistrationGuard(
      "00000000000000000000000000000010", "00000000000000000000000000000001", "00000000000000000000000000000002",
      "00000000000000000000000000000003", "00000000000000000000000000000004", "00000000000000000000000000000005",
      "00000000000000000000000000000006", "00000000000000000000000000000007", "00000000000000000000000000000008",
      "00000000000000000000000000000009", "0000000000000000000000000000000a");

  sleep(10);

  return RUN_ALL_TESTS();
}