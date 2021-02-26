#include "mars_routing_common/topology/Connection.h"

mars::routing::common::topology::Connection::Connection(
    mars::routing::common::topology::Vertex& pOrigin,
    mars::routing::common::topology::Vertex& pDestination)
    : mOrigin(pOrigin), mDestination(pDestination)
{
}

mars::routing::common::topology::Connection::Connection(
    mars_topology_msgs::Connection& pMessage)
    : mOrigin(mars::common::Id(pMessage.origin_id)),
      mDestination(mars::common::Id(pMessage.destination_id))
{
}

mars::routing::common::topology::Vertex&
mars::routing::common::topology::Connection::getOrigin()
{
  return this->mOrigin;
}

mars::routing::common::topology::Vertex&
mars::routing::common::topology::Connection::getDestination()
{
  return this->mDestination;
}
