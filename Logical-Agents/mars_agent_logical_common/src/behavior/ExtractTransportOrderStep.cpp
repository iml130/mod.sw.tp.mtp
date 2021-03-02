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

#include "mars_agent_logical_common/behavior/ExtractTransportOrderStep.h"

static const std::string
    BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_ORDER = "transport_order_step";
static const std::string
    BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_MOVE_ORDER = "move_order";
static const std::string
    BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_ROBOT_ACTION =
        "robot_action";

mars::agent::logical::common::behavior::ExtractTransportOrderStep::
    ExtractTransportOrderStep(const std::string& pName,
                              const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
}

BT::PortsList mars::agent::logical::common::behavior::
    ExtractTransportOrderStep::providedPorts()
{
  return {// input: TransportOrderStep
          BT::InputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
              BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_ORDER),
          // output: MoveOrder
          BT::OutputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
              BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_MOVE_ORDER),
          // output: RobotAction
          BT::OutputPort<
              std::shared_ptr<mars::agent::logical::common::RobotAction>>(
              BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_ROBOT_ACTION)};
}

BT::NodeStatus
mars::agent::logical::common::behavior::ExtractTransportOrderStep::tick()
{
  std::shared_ptr<mars::agent::logical::common::TransportOrderStep>
      transportOrderStep;

  BT::Optional<std::shared_ptr<mars::agent::logical::common::Order>> order;
  BT::Result result;

  // read order from blackboard
  order = this->getInput<std::shared_ptr<mars::agent::logical::common::Order>>(
      BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_ORDER);
  if (!order)
  {
    MARS_LOG_ERROR(order.error());
    return BT::NodeStatus::FAILURE;
  }

  // cast to transport order step
  transportOrderStep = std::dynamic_pointer_cast<
      mars::agent::logical::common::TransportOrderStep>(order.value());
  if (!transportOrderStep)
  {
    // it was not a transport order step
    return BT::NodeStatus::FAILURE;
  }

  // extract move order
  result =
      this->setOutput<std::shared_ptr<mars::agent::logical::common::Order>>(
          BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_MOVE_ORDER,
          std::make_shared<mars::agent::logical::common::MoveOrder>(
              transportOrderStep->getMoveOrder()));

  if (!result)
  {
    MARS_LOG_ERROR(result.error());
    return BT::NodeStatus::FAILURE;
  }
  // extract robot action
  result = this->setOutput<
      std::shared_ptr<mars::agent::logical::common::RobotAction>>(
      BEHAVIOR_EXTRACT_TRANSPORT_ORDER_STEP_PARAM_NAME_ROBOT_ACTION,
      std::make_shared<mars::agent::logical::common::RobotAction>(
          transportOrderStep->getRobotAction()));

  if (!result)
  {
    MARS_LOG_ERROR(result.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
