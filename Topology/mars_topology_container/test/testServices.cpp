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

// include utility
#include <mars_common/Id.h>
#include <mars_common/TimeInterval.h>
#include <mars_topology_common/utility/RegistrationGuard.hpp>

// include services
#include <mars_topology_srvs/AddReservation.h>
#include <mars_topology_srvs/DeallocateEntity.h>
#include <mars_topology_srvs/DeleteReservation.h>
#include <mars_topology_srvs/GetCoordinate.h>
#include <mars_topology_srvs/GetFootprint.h>
#include <mars_topology_srvs/GetFreeTimeSlots.h>
#include <mars_topology_srvs/GetIngoingEdges.h>
#include <mars_topology_srvs/GetOutgoingEdges.h>
#include <mars_topology_srvs/GetRestrictions.h>
#include <mars_topology_srvs/GetStatus.h>
#include <mars_topology_srvs/GetType.h>
#include <mars_topology_srvs/LockTopologyEntity.h>
#include <mars_topology_srvs/UnlockTopologyEntity.h>

TEST(ServicesTests, getCoordinate)
{
  mars_topology_srvs::GetCoordinate foo;

mars::common::Id entity("00000000000000000000000000000001","vertex");
  foo.request.entity_id=entity.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/container_afd0b036625a3aa8b6399dc8c8fff0ff/get_coordinate", foo));

      EXPECT_EQ(foo.response.point.point.x,1);
      EXPECT_EQ(foo.response.point.point.y,1);

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestContainerServices");

  mars::topology::common::utility::RegistrationGuard("afd0b036625a3aa8b6399dc8c8fff0ff");
//sleep(5);
  return RUN_ALL_TESTS();
}