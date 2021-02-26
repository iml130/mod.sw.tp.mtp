#include "mars_routing_common/topology/Status.h"

mars::routing::common::topology::Status::Status(
    const std::vector<mars::topology::common::TopologyEntityLock>& pLocks,
    const std::vector<mars::topology::common::TopologyEntityReservation>& pReservations)
  : mLocks(pLocks), mReservations(pReservations)
{
}

const std::vector<mars::topology::common::TopologyEntityLock>&
mars::routing::common::topology::Status::getLocks() const
{
  return mLocks;
}

const std::vector<mars::topology::common::TopologyEntityReservation>&
mars::routing::common::topology::Status::getReservations() const
{
  return mReservations;
}