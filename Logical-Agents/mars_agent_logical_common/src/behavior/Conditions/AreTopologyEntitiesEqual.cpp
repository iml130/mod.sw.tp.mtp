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

#include "mars_agent_logical_common/behavior/Conditions/AreTopologyEntitiesEqual.h"

static const std::string BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_NAME_FIRST = "first_entity";
static const std::string BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_DESCRIPTION_FIRST =
    "First entity to check for equal.";

static const std::string BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_NAME_SECOND = "second_entity";
static const std::string BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_DESCRIPTION_SECOND =
    "Second entity to check for equal.";

mars::agent::logical::common::behavior::AreTopologyEntitiesEqual::
    AreTopologyEntitiesEqual(const std::string& pName,
                             const BT::NodeConfiguration& pConfig)
    : BT::ConditionNode(pName, pConfig)
{
}

BT::PortsList mars::agent::logical::common::behavior::AreTopologyEntitiesEqual::providedPorts()
{
  return {// First std::shared_ptr<mars::routing::common::topology::Entity>
          BT::InputPort<std::shared_ptr<mars::routing::common::topology::Entity>>(
              BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_NAME_FIRST,
              BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_DESCRIPTION_FIRST),
          // Second std::shared_ptr<mars::routing::common::topology::Entity>
          BT::InputPort<std::shared_ptr<mars::routing::common::topology::Entity>>(
              BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_NAME_SECOND,
              BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_DESCRIPTION_SECOND)

  };
}

BT::NodeStatus mars::agent::logical::common::behavior::AreTopologyEntitiesEqual::tick()
{
  BT::Optional<std::shared_ptr<mars::routing::common::topology::Entity>> first, second;

  first =
      this->getInput<std::shared_ptr<mars::routing::common::topology::Entity>>(BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_NAME_FIRST);
  if (!first)
  {
    MARS_LOG_ERROR(first.error());
    return BT::NodeStatus::FAILURE;
  }

  second =
      this->getInput<std::shared_ptr<mars::routing::common::topology::Entity>>(BEHAVIOR_ARETOPOLOGYENTITIESEQUAL_PARAM_NAME_SECOND);
  if (!second)
  {
    MARS_LOG_ERROR(second.error());
    return BT::NodeStatus::FAILURE;
  }

  if (*(first.value()) == *(second.value()))
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
