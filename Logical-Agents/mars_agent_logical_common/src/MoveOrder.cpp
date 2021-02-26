/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : MARS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

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
