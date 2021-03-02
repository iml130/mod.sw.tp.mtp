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

#include "mars_agent_logical_common/behavior/Conditions/IsMoveOrder.h"

static const std::string BEHAVIOR_ISMOVEORDER_PARAM_NAME_ORDER =
    "order";
static const std::string BEHAVIOR_ISMOVEORDER_PARAM_DESCRIPTION_ORDER =
    "Order which should be tested, if it is a move order.";

mars::agent::logical::common::behavior::IsMoveOrder::IsMoveOrder(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::ConditionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::IsMoveOrder::providedPorts()
{
  return {// Order which shall be tested on being a move order
          BT::InputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
              BEHAVIOR_ISMOVEORDER_PARAM_NAME_ORDER,
              BEHAVIOR_ISMOVEORDER_PARAM_DESCRIPTION_ORDER)};
}

BT::NodeStatus mars::agent::logical::common::behavior::IsMoveOrder::tick()
{
  std::shared_ptr<mars::agent::logical::common::MoveOrder> moveOrder;
  BT::Optional<std::shared_ptr<mars::agent::logical::common::Order>> order;

  order = this->getInput<std::shared_ptr<mars::agent::logical::common::Order>>(
      BEHAVIOR_ISMOVEORDER_PARAM_NAME_ORDER);
  if (!order)
  {
    MARS_LOG_ERROR(order.error());
    return BT::NodeStatus::FAILURE;
  }

  moveOrder =
      std::dynamic_pointer_cast<mars::agent::logical::common::MoveOrder>(
          order.value());
  if (!moveOrder)
  {
    // its not a move order
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
