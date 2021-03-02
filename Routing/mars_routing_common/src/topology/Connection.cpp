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
