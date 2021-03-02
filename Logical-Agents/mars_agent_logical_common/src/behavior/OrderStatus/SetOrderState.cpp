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

#include "mars_agent_logical_common/behavior/OrderStatus/SetOrderState.h"

static const std::string BEHAVIOR_ORDER_STATE_READ_PARAM_NAME =
    "input_order_state";
static const std::string BEHAVIOR_ORDER_STATE_READ_PARAMETER_DESCRIPTION = "";

static const std::string BEHAVIOR_ORDER_STATE_WRITE_PARAM_NAME = "order_state";
static const std::string BEHAVIOR_ORDER_STATE_WRITE_PARAMETER_DESCRIPTION = "";

mars::agent::logical::common::behavior::SetOrderState::SetOrderState(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
  this->mStateEnums.insert(

      {{"ORDER_STATE_UNKNOWN",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_UNKNOWN},
       // Move order
       {"ORDER_STATE_MO_MOVE_ORDER_UNAVAILABLE",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_MO_MOVE_ORDER_UNAVAILABLE},
       {"ORDER_STATE_MO_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_MO_MOVE_ORDER_START},
       {"ORDER_STATE_MO_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_MO_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_MO_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_MO_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_MO_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_MO_MOVE_ORDER_ERROR},
       // Transport order step
       {"ORDER_STATE_TOS_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_MOVE_ORDER_START},
       {"ORDER_STATE_TOS_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TOS_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_TOS_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TOS_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_TOS_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_MOVE_ORDER_ERROR},
       {"ORDER_STATE_TOS_ACTION_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_START},
       {"ORDER_STATE_TOS_ACTION_ONGOING",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_ONGOING},
       {"ORDER_STATE_TOS_ACTION_FINISHED",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_FINISHED},
       {"ORDER_STATE_TOS_ACTION_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_ERROR},
       // Transport order
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_START},
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_ERROR},
       {"ORDER_STATE_TO_LOAD_ACTION_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TO_LOAD_ACTION_START},
       {"ORDER_STATE_TO_LOAD_ACTION_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_ACTION_ONGOING},
       {"ORDER_STATE_TO_LOAD_ACTION_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_ACTION_FINISHED},
       {"ORDER_STATE_TO_LOAD_ACTION_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TO_LOAD_ACTION_ERROR},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_START},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ERROR},
       {"ORDER_STATE_TO_UNLOAD_ACTION_START",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_START},
       {"ORDER_STATE_TO_UNLOAD_ACTION_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_ONGOING},
       {"ORDER_STATE_TO_UNLOAD_ACTION_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_FINISHED},
       {"ORDER_STATE_TO_UNLOAD_ACTION_ERROR",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_ERROR}

      });
}

BT::PortsList
mars::agent::logical::common::behavior::SetOrderState::providedPorts()
{
  return {// Order status
          BT::InputPort<std::string>(
              BEHAVIOR_ORDER_STATE_READ_PARAM_NAME,
              BEHAVIOR_ORDER_STATE_READ_PARAMETER_DESCRIPTION),
          BT::OutputPort<unsigned int>(
              BEHAVIOR_ORDER_STATE_WRITE_PARAM_NAME,
              BEHAVIOR_ORDER_STATE_WRITE_PARAMETER_DESCRIPTION)};
}

BT::NodeStatus mars::agent::logical::common::behavior::SetOrderState::tick()
{
  BT::Result writeBackResult;
  BT::Optional<std::string> readResult;
  std::string inOrderState;
  unsigned int outOrderState;

  // get path id
  readResult = getInput<std::string>(BEHAVIOR_ORDER_STATE_READ_PARAM_NAME);
  if (!readResult)
  {
    // could not read the order status, which should be set
    MARS_LOG_ERROR(readResult.error());
    return BT::NodeStatus::FAILURE;
  }
  inOrderState = readResult.value();

  outOrderState = this->mStateEnums[inOrderState];

  // update order status on blackboard
  writeBackResult = setOutput<unsigned int>(
      BEHAVIOR_ORDER_STATE_WRITE_PARAM_NAME, outOrderState);

  // log error from setting output
  if (!writeBackResult)
  {
    MARS_LOG_ERROR(writeBackResult.error());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
