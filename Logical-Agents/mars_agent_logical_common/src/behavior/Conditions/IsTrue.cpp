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

#include "mars_agent_logical_common/behavior/Conditions/IsTrue.h"

static const std::string PARAM_NAME_FLAG = "input";

mars::agent::logical::common::behavior::IsTrue::IsTrue(const std::string& pName,
                                                       const BT::NodeConfiguration& pConfig)
    : BT::ConditionNode(pName, pConfig)
{
}

BT::PortsList mars::agent::logical::common::behavior::IsTrue::providedPorts()
{
  return {BT::InputPort<bool>(PARAM_NAME_FLAG)};
}

BT::NodeStatus mars::agent::logical::common::behavior::IsTrue::tick()
{
  BT::Optional<bool> lFlag = this->getInput<bool>(PARAM_NAME_FLAG);

  if (!lFlag)
  {
    MARS_LOG_ERROR(lFlag.error());
    return BT::NodeStatus::FAILURE;
  }

  return (lFlag.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
