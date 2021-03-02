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

#ifndef TRANSPORTORDER_H
#define TRANSPORTORDER_H

#include "mars_agent_logical_common/Order.h"
#include "mars_agent_logical_common/TransportOrderStep.h"

#include <mars_agent_logical_msgs/TransportOrder.h>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
class TransportOrder : public mars::agent::logical::common::Order
{
public:
  TransportOrder(
      const mars_agent_logical_msgs::TransportOrder& transportOrderMsg);

  const mars::agent::logical::common::TransportOrderStep& getStartStep() const;

  const mars::agent::logical::common::TransportOrderStep&
  getDestinationStep() const;

private:
  mars::agent::logical::common::TransportOrderStep mStartStep;

  mars::agent::logical::common::TransportOrderStep mDestinationStep;
};
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // TRANSPORTORDER_H
