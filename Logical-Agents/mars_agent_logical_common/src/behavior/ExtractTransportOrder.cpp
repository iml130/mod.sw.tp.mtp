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

#include "mars_agent_logical_common/behavior/ExtractTransportOrder.h"

static const std::string BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_ORDER =
    "order";
static const std::string
    BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_FIRST_STEP = "first_step";
static const std::string
    BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_SECOND_STEP = "second_step";

mars::agent::logical::common::behavior::ExtractTransportOrder::
    ExtractTransportOrder(const std::string& pName,
                          const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::ExtractTransportOrder::providedPorts()
{
  return {
      BT::InputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
          BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_ORDER),
      BT::OutputPort<
          std::shared_ptr<mars::agent::logical::common::Order>>(
          BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_FIRST_STEP),
      BT::OutputPort<
          std::shared_ptr<mars::agent::logical::common::Order>>(
          BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_SECOND_STEP)};
}

BT::NodeStatus
mars::agent::logical::common::behavior::ExtractTransportOrder::tick()
{

  std::shared_ptr<mars::agent::logical::common::TransportOrder> transportOrder;

  BT::Optional<std::shared_ptr<mars::agent::logical::common::Order>> order;
  BT::Result result;

  // read order from blackboard
  order = this->getInput<std::shared_ptr<mars::agent::logical::common::Order>>(
      BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_ORDER);
  if (!order)
  {
    MARS_LOG_ERROR(order.error());
    return BT::NodeStatus::FAILURE;
  }

  // cast to transport order
  transportOrder =
      std::dynamic_pointer_cast<mars::agent::logical::common::TransportOrder>(
          order.value());
  if (!transportOrder)
  {
    // it was not a transport order
    return BT::NodeStatus::FAILURE;
  }

  // extract first step
    result = this->setOutput<
        std::shared_ptr<mars::agent::logical::common::Order>>(
        BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_FIRST_STEP,
        std::make_shared<mars::agent::logical::common::TransportOrderStep>(
            transportOrder->getStartStep()));

    if (!result)
    {
      MARS_LOG_ERROR(result.error());
      return BT::NodeStatus::FAILURE;
    }
  // extract second step
    result = this->setOutput<
        std::shared_ptr<mars::agent::logical::common::Order>>(
        BEHAVIOR_EXTRACT_TRANSPORT_ORDER_PARAM_NAME_SECOND_STEP,
        std::make_shared<mars::agent::logical::common::TransportOrderStep>(
            transportOrder->getDestinationStep()));

    if (!result)
    {
      MARS_LOG_ERROR(result.error());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}
