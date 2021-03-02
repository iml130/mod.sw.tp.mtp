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

#include "mars_agent_logical_common/behavior/Conditions/IsTransportOrder.h"

static const std::string BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_NAME_ORDER =
    "order";
static const std::string BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_DESCRIPTION_ORDER =
    "Order which should be tested, if it is a transport order.";

mars::agent::logical::common::behavior::IsTransportOrder::IsTransportOrder(const std::string &pName, const BT::NodeConfiguration &pConfig): BT::ConditionNode(pName, pConfig)
{

}

BT::PortsList mars::agent::logical::common::behavior::IsTransportOrder::providedPorts()
{
  return {// Order which shall be tested on being a transport order
          BT::InputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
              BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_NAME_ORDER,
              BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_DESCRIPTION_ORDER)};
}

BT::NodeStatus mars::agent::logical::common::behavior::IsTransportOrder::tick()
{
  std::shared_ptr<mars::agent::logical::common::TransportOrder> transportOrder;
  BT::Optional<std::shared_ptr<mars::agent::logical::common::Order>> order;

  order = this->getInput<std::shared_ptr<mars::agent::logical::common::Order>>(
      BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_NAME_ORDER);
  if (!order)
  {
    MARS_LOG_ERROR(order.error());
    return BT::NodeStatus::FAILURE;
  }

  transportOrder =
      std::dynamic_pointer_cast<mars::agent::logical::common::TransportOrder>(
          order.value());
  if (!transportOrder)
  {
    // its not a move order
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
