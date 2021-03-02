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

#include "mars_agent_logical_common/behavior/Conditions/IsManualAction.h"

static const std::string PARAM_NAME_ROBOT_ACTION = "robot_action";

mars::agent::logical::common::behavior::IsManualAction::IsManualAction(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::ConditionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::IsManualAction::providedPorts()
{
  return {
      // Action
      BT::InputPort<std::shared_ptr<mars::agent::logical::common::RobotAction>>(
          PARAM_NAME_ROBOT_ACTION)};
}

BT::NodeStatus mars::agent::logical::common::behavior::IsManualAction::tick()
{
  BT::Optional<std::shared_ptr<mars::agent::logical::common::RobotAction>>
      lAction = this->getInput<
          std::shared_ptr<mars::agent::logical::common::RobotAction>>(
          PARAM_NAME_ROBOT_ACTION);

  if (!lAction)
  {
    MARS_LOG_ERROR(lAction.error());
    return BT::NodeStatus::FAILURE;
  }

  if (lAction.value()->isManual())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
