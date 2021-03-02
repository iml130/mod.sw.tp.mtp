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

#ifndef MARS_AGENT_LOGICAL_COMMON_ORDER_H
#define MARS_AGENT_LOGICAL_COMMON_ORDER_H

#include "mars_common/Id.h"
#include "mars_common/Logger.h"
#include "mars_topology_common/TopologyEntity.h"

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
/**
 * @brief The order class serves as a superclass for any kind of orders.
 */
class Order
{
public:
  Order(const mars::common::Id& pOrderId);

  virtual ~Order();

  mars::common::Id getId(void) const;

  ros::Time getCreationTime(void) const;
  ros::Time getStartTime(void) const;
  ros::Time getCompletionTime(void) const;

  bool isCompleted(void) const;

protected:
  mars::common::Id mId;

  ros::Time mCreationTime;
  ros::Time mStartTime;
  ros::Time mCompletionTime;
};
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_ORDER_H
