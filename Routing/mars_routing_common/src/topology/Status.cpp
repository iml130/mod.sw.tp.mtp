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