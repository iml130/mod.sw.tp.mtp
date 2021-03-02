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

#include "mars_agent_logical_common/behavior/Decorators/RepeatOncePerTick.h"

mars::agent::logical::common::behavior::RepeatOncePerTick::RepeatOncePerTick(
    const std::string& pName)
    : BT::DecoratorNode(pName, {})
{
  setRegistrationID("RepeatOnce");
}

BT::NodeStatus mars::agent::logical::common::behavior::RepeatOncePerTick::tick()
{
  BT::NodeStatus child_state = this->child_node_->executeTick();
  BT::NodeStatus nodeStatus;

  switch (child_state)
  {
  case BT::NodeStatus::SUCCESS:
    nodeStatus = BT::NodeStatus::RUNNING;
    break;

  case BT::NodeStatus::FAILURE:
    nodeStatus = (BT::NodeStatus::FAILURE);
    break;

  case BT::NodeStatus::RUNNING:
    nodeStatus = BT::NodeStatus::RUNNING;
    break;

  default:
    throw BT::LogicError("A child node must never return IDLE");
  }
  return nodeStatus;
}

void mars::agent::logical::common::behavior::RepeatOncePerTick::halt()
{
  BT::DecoratorNode::halt();
}
