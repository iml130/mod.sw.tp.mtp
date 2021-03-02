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

#include "mars_agent_logical_common/behavior/Decorators/RepeatUntilSuccessful.h"

mars::agent::logical::common::behavior::RepeatUntilSuccessful::
    RepeatUntilSuccessful(const std::string& pName)
    : BT::DecoratorNode(pName, {})
{
  setRegistrationID("RepeatOnce");
}

void mars::agent::logical::common::behavior::RepeatUntilSuccessful::halt()
{
  BT::DecoratorNode::halt();
}

BT::NodeStatus mars::agent::logical::common::behavior::RepeatUntilSuccessful::tick()
{
  BT::NodeStatus child_state = this->child_node_->executeTick();
  BT::NodeStatus nodeStatus;

  switch (child_state)
  {
  case BT::NodeStatus::SUCCESS:
    nodeStatus = BT::NodeStatus::SUCCESS;
    break;

  case BT::NodeStatus::FAILURE:
    nodeStatus = BT::NodeStatus::RUNNING;
    break;

  case BT::NodeStatus::RUNNING:
    nodeStatus = BT::NodeStatus::RUNNING;
    break;

  default:
    throw BT::LogicError("A child node must never return IDLE");
  }
  return nodeStatus;
}
