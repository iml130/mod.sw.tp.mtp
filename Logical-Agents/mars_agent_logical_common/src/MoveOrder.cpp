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


#include "mars_agent_logical_common/MoveOrder.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

mars::agent::logical::common::MoveOrder::MoveOrder(
    const mars_agent_logical_msgs::MoveOrder& pMoveOrderMsg)
    : mars::agent::logical::common::Order(pMoveOrderMsg.move_order_id),
      mDestinationReservationDuration(pMoveOrderMsg.destination_reservation_time)
{
  if (pMoveOrderMsg.destination_entity.entity_type.entity_type >=
          pMoveOrderMsg.destination_entity.entity_type.TOPOLOGY_VERTEX_TYPE_MIN &&
      pMoveOrderMsg.destination_entity.entity_type.entity_type <=
          pMoveOrderMsg.destination_entity.entity_type.TOPOLOGY_VERTEX_TYPE_MAX)
  {
    mDestination = std::make_shared<mars::routing::common::topology::Vertex>(pMoveOrderMsg.destination_entity.id);
  }
  else if (pMoveOrderMsg.destination_entity.entity_type.entity_type >=
               pMoveOrderMsg.destination_entity.entity_type.TOPOLOGY_EDGE_TYPE_MIN &&
           pMoveOrderMsg.destination_entity.entity_type.entity_type <=
               pMoveOrderMsg.destination_entity.entity_type.TOPOLOGY_EDGE_TYPE_MAX)
  {
    mDestination = std::make_shared<mars::routing::common::topology::Edge>(pMoveOrderMsg.destination_entity.id);
  }
  else
  {
    // TODO: Error, unknown topology entity!
  }
}

mars::agent::logical::common::MoveOrder::~MoveOrder()
{
}

std::shared_ptr<mars::routing::common::topology::Entity>
mars::agent::logical::common::MoveOrder::getDestination() const
{
  return mDestination;
}

const ros::Duration&
mars::agent::logical::common::MoveOrder::getDestinationReservationDuration() const
{
  return mDestinationReservationDuration;
}
