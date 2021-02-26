#ifndef MARS_ROUTING_COMMON_TOPOLOGY_STATUS_H
#define MARS_ROUTING_COMMON_TOPOLOGY_STATUS_H

#include "mars_topology_common/TopologyEntityLock.h"
#include "mars_topology_common/TopologyEntityReservation.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
class Status
{
public:
  /**
   * @brief Status Default Constructor
   */
  Status(const std::vector<mars::topology::common::TopologyEntityLock>& pLocks,
         const std::vector<mars::topology::common::TopologyEntityReservation>& pReservations);

  /**
   * @brief Primitive getter for the vector of Locks;
   * 
   * @return const reference to the vector of Locks;
   */
  const std::vector<mars::topology::common::TopologyEntityLock>& getLocks() const;

  /**
   * @brief Primitive getter for the vector of Reservations;
   * 
   * @return const reference to the vector of Reservations;
   */
  const std::vector<mars::topology::common::TopologyEntityReservation>& getReservations() const;

private:
  std::vector<mars::topology::common::TopologyEntityLock> mLocks;
  std::vector<mars::topology::common::TopologyEntityReservation> mReservations;
};
}  // namespace topology
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_TOPOLOGY_STATUS_H
