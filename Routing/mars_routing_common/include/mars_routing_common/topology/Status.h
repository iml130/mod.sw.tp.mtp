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
