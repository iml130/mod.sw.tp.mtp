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

#include "mars_agent_logical_common/TransportOrder.h"

mars::agent::logical::common::TransportOrder::TransportOrder(
    const mars_agent_logical_msgs::TransportOrder& transportOrderMsg)
    : mars::agent::logical::common::Order(transportOrderMsg.transport_order_id), mStartStep(transportOrderMsg.start_step), mDestinationStep(transportOrderMsg.destination_step)
{
}

const mars::agent::logical::common::TransportOrderStep&
mars::agent::logical::common::TransportOrder::getDestinationStep() const
{
  return mDestinationStep;
}

const mars::agent::logical::common::TransportOrderStep&
mars::agent::logical::common::TransportOrder::getStartStep() const
{
  return mStartStep;
}
