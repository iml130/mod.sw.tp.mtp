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

#include "mars_agent_logical_common/behavior/Conditions/AreIdsEqual.h"

static const std::string BEHAVIOR_AREIDSEQUAL_PARAM_NAME_FIRST = "first_id";
static const std::string BEHAVIOR_AREIDSEQUAL_PARAM_DESCRIPTION_FIRST =
    "First id to check for equal.";

static const std::string BEHAVIOR_AREIDSEQUAL_PARAM_NAME_SECOND = "second_id";
static const std::string BEHAVIOR_AREIDSEQUAL_PARAM_DESCRIPTION_SECOND =
    "Second id to check for equal.";

mars::agent::logical::common::behavior::AreIdsEqual::AreIdsEqual(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::ConditionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::AreIdsEqual::providedPorts()
{
  return {// First mars::common::Id
          BT::InputPort<mars::common::Id>(
              BEHAVIOR_AREIDSEQUAL_PARAM_NAME_FIRST,
              BEHAVIOR_AREIDSEQUAL_PARAM_DESCRIPTION_FIRST),
          // Second mars::common::Id
          BT::InputPort<mars::common::Id>(
              BEHAVIOR_AREIDSEQUAL_PARAM_NAME_SECOND,
              BEHAVIOR_AREIDSEQUAL_PARAM_DESCRIPTION_SECOND)

  };
}

BT::NodeStatus mars::agent::logical::common::behavior::AreIdsEqual::tick()
{
  BT::Optional<mars::common::Id> first, second;

  first =
      this->getInput<mars::common::Id>(BEHAVIOR_AREIDSEQUAL_PARAM_NAME_FIRST);
  if (!first)
  {
    MARS_LOG_ERROR(first.error());
    return BT::NodeStatus::FAILURE;
  }

  second =
      this->getInput<mars::common::Id>(BEHAVIOR_AREIDSEQUAL_PARAM_NAME_SECOND);
  if (!second)
  {
    MARS_LOG_ERROR(second.error());
    return BT::NodeStatus::FAILURE;
  }

  if (first == second)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
