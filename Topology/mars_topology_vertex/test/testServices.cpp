// include core libs
#include <gtest/gtest.h>
#include <ros/ros.h>

// include utility
#include <actionlib/client/simple_action_client.h>
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



// These tests are not status dependend

TEST(ServicesTests, getCoordinate)
{
  mars_topology_srvs::GetCoordinate foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_afd0b036625a3aa8b6399dc8c8fff0ff/get_coordinate", foo));

  EXPECT_EQ(foo.response.point.point.x, 4);
  EXPECT_EQ(foo.response.point.point.y, 5);
}

TEST(ServicesTests, getFootprint)
{
  mars_topology_srvs::GetFootprint foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_afd0b036625a3aa8b6399dc8c8fff0ff/get_footprint", foo));

  EXPECT_EQ(foo.response.footprint.polygon.points[0].x, -2);
  EXPECT_EQ(foo.response.footprint.polygon.points[0].y, 2);

  EXPECT_EQ(foo.response.footprint.polygon.points[1].x, 2);
  EXPECT_EQ(foo.response.footprint.polygon.points[1].y, 2);

  EXPECT_EQ(foo.response.footprint.polygon.points[2].x, 2);
  EXPECT_EQ(foo.response.footprint.polygon.points[2].y, -2);

  EXPECT_EQ(foo.response.footprint.polygon.points[3].x, -2);
  EXPECT_EQ(foo.response.footprint.polygon.points[3].y, -2);
}

TEST(ServicesTests, getIngoingEdges)
{
  mars_topology_srvs::GetIngoingEdges foo;

  mars::common::Id isId;
  mars::common::Id setId("00000000-0000-0000-0000-000000000002");

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_afd0b036625a3aa8b6399dc8c8fff0ff/get_ingoing_edges",
      foo));

  isId = mars::common::Id::convertToMsgId(foo.response.ingoing_edges_ids[0]);

  EXPECT_EQ(setId.getUUID(), isId.getUUID());
}

TEST(ServicesTests, getOutgoingEdges)
{
  mars_topology_srvs::GetOutgoingEdges foo;

  mars::common::Id isId;
  mars::common::Id setId("00000000-0000-0000-0000-000000000002");

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_afd0b036625a3aa8b6399dc8c8fff0ff/get_outgoing_edges",
      foo));

  isId = mars::common::Id::convertToMsgId(foo.response.outgoing_edges_ids[0]);

  EXPECT_EQ(setId.getUUID(), isId.getUUID());
}

TEST(ServicesTests, getType)
{
  mars_topology_srvs::GetType foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_afd0b036625a3aa8b6399dc8c8fff0ff/get_type", foo));

  EXPECT_EQ(
      foo.response.entity_type.entity_type,
      mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT);
}

TEST(ServicesTests, getRestrictions)
{
  mars_topology_srvs::GetRestrictions foo;

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_afd0b036625a3aa8b6399dc8c8fff0ff/get_restrictions",
      foo));

  EXPECT_EQ(foo.response.max_angular_velocity, 2);
  EXPECT_EQ(foo.response.max_angular_acceleration, 1.5);
  EXPECT_EQ(foo.response.max_height, 5);
  EXPECT_EQ(foo.response.max_linear_acceleration, 2);
  EXPECT_EQ(foo.response.max_linear_velocity, 15);
  EXPECT_EQ(foo.response.max_total_weight, 150);

  EXPECT_TRUE(foo.response.forbidden_vehicle_types.empty());
  EXPECT_TRUE(foo.response.forbidden_hazard_types.empty());
}

//These tests are status depended

TEST(ServicesTests, getStatus)
{
  mars_topology_srvs::AddReservation addRes;
  mars_topology_srvs::DeleteReservation delRes;

  mars_topology_srvs::GetStatus status;

  mars::common::Id agentId, pathId;
  mars::common::TimeInterval resInterval(ros::Time::now(), ros::Duration(10));

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe2/add_reservation",
      addRes));

  // Get Status
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe2/get_status", status));

  EXPECT_TRUE(status.response.locks.empty());
  EXPECT_EQ(status.response.reservations.size(), 1);
  EXPECT_EQ(mars::common::Id(status.response.reservations[0].agent_id),
            agentId);
  EXPECT_EQ(mars::common::Id(status.response.reservations[0].path_id), pathId);
  EXPECT_EQ(
      mars::common::TimeInterval(status.response.reservations[0].time_interval),
      resInterval);

  // Delete Reservation
  delRes.request.agent_id = agentId.toMsg();
  delRes.request.path_id = pathId.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe2/delete_reservation",
      delRes));

  EXPECT_EQ(delRes.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  // Get Status
  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe2/get_status", status));

  EXPECT_TRUE(status.response.locks.empty());
  EXPECT_TRUE(status.response.reservations.empty());
}

TEST(ServicesTests, addOneReservation)
{
  mars_topology_srvs::AddReservation addRes;
  mars::common::TimeInterval resInterval(ros::Time::now(), ros::Duration(10));
  mars::common::Id agentId, pathId;

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_afd0b036625a3aa8b6399dc8c8fff0ff/add_reservation",
      addRes));

  EXPECT_EQ(addRes.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);
}

TEST(ServicesTests, freeTimeSlotOneReservation)
{
  mars_topology_srvs::AddReservation addRes;
  mars::common::TimeInterval resInterval(ros::Time::now(), ros::Duration(10));
  mars::common::Id agentId, pathId;
  mars_topology_srvs::GetFreeTimeSlots freeSlots;

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe1/add_reservation",
      addRes));

  EXPECT_EQ(addRes.response.result.result,
            mars_common_msgs::Result::RESULT_SUCCESS);

  // Check free time windows
  freeSlots.request.start_time = ros::Time(0);

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe1/get_free_time_slots",
      freeSlots));

  EXPECT_EQ(freeSlots.response.time_interval.size(), 2);

  mars::common::TimeInterval freeSlotOne(freeSlots.response.time_interval[0]);
  mars::common::TimeInterval freeSlotTwo(freeSlots.response.time_interval[1]);

  EXPECT_EQ(freeSlotOne, mars::common::TimeInterval(
                             ros::Time(0), resInterval.getStartTime()));
  EXPECT_EQ(freeSlotTwo, mars::common::TimeInterval(
                             resInterval.getEndTime(),
                             mars::common::TimeInterval::INFINITE_DURATION));
}

TEST(ServicesTests, deleteReservation)
{
  mars_topology_srvs::AddReservation addRes;
  mars_topology_srvs::DeleteReservation delRes;
  mars_topology_srvs::GetFreeTimeSlots freeSlots;

  mars::common::TimeInterval resInterval(ros::Time(210), ros::Duration(10));
  mars::common::Id agentId, pathId;

  agentId.initialize();
  pathId.initialize();

  // Add a reservation
  addRes.request.reservation.agent_id = agentId.toMsg();
  addRes.request.reservation.path_id = pathId.toMsg();
  addRes.request.reservation.time_interval = resInterval.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_4c25ff6fa99d35f99230687818b86412/add_reservation",
      addRes));

  // delete the previous reservation
  delRes.request.agent_id = agentId.toMsg();
  delRes.request.path_id = pathId.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_4c25ff6fa99d35f99230687818b86412/delete_reservation",
      delRes));
}

TEST(ServiceTests, deallocateEntityWithoutAllocate)
{
  mars::common::Id agentId, pathId;
  mars_topology_srvs::DeallocateEntity del;

  agentId.initialize();
  pathId.initialize();

  // deallocate
  del.request.agent_id = agentId.toMsg();

  EXPECT_TRUE(ros::service::call(
      "/topology/vertex_1950225e86e33e1394c61c4fa5bcafe3/deallocate", del));

  EXPECT_EQ(del.response.result.result, mars_common_msgs::Result::RESULT_ERROR);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestVertexServices");

  mars::topology::common::utility::RegistrationGuard(
      "afd0b036625a3aa8b6399dc8c8fff0ff", "1950225e86e33e1394c61c4fa5bcafe1",
      "4c25ff6fa99d35f99230687818b86412", "1950225e86e33e1394c61c4fa5bcafe2",
      "1950225e86e33e1394c61c4fa5bcafe3");

  return RUN_ALL_TESTS();
}
