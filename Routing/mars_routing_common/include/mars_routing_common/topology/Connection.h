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

#ifndef MARS_ROUTING_COMMON_TOPOLOGY_CONNECTION_H
#define MARS_ROUTING_COMMON_TOPOLOGY_CONNECTION_H

#include "mars_common/Id.h"
#include "mars_common/TimeInterval.h"

#include "mars_routing_common/topology/Vertex.h"
#include "mars_topology_msgs/Connection.h"

#include <eigen3/Eigen/Core>

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
class Connection
{
public:
  /**
   * @brief Direct constructor for a connection between two topology vertices
   * @param pOrigin Origin vertex of the connection
   * @param pDestination Destination vertex of the connection
   */
  Connection(mars::routing::common::topology::Vertex& pOrigin, mars::routing::common::topology::Vertex& pDestination);

  /**
   * @brief Constructor from Connection message
   * @param pMessage Ingoing connection message to construct from
   */
  Connection(mars_topology_msgs::Connection& pMessage);

  /**
   * @brief Primitive getter for mOrigin
   * @return The origin vertex of the connection
   */
  mars::routing::common::topology::Vertex& getOrigin();

  /**
   * @brief Primitive getter for mDestination
   * @return The destination vertex of the connection
   */
  mars::routing::common::topology::Vertex& getDestination();

private:
  mars::routing::common::topology::Vertex mOrigin;
  mars::routing::common::topology::Vertex mDestination;
};
}  // namespace topology
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_TOPOLOGY_CONNECTION_H
