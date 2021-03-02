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

#include "mars_agent_logical_common/behavior/ImportROSParameter.h"
#include "mars_common/Logger.h"

static const std::string SERVICE_NAME_GET_ROUTE = "/GetRoute";

BT::NodeStatus mars::agent::logical::common::behavior::ImportROSParameter::tick()
{
  std::string lValue;
  BT::Optional<std::string> lParameterName;
  BT::Result lResult;

  lParameterName = this->getInput<std::string>(BEHAVIOR_IMPORTROSPARAMETER_PARAM_NAME);
  if (!lParameterName)
  {
    MARS_LOG_ERROR(lParameterName.error());
    return BT::NodeStatus::FAILURE;
  }

  if (!this->mNHPriv.getParam(lParameterName.value(), lValue))
  {
    MARS_LOG_ERROR_GET_ROS_PARAMETER(lParameterName.value());
    return BT::NodeStatus::FAILURE;
  };

  lResult = this->setOutput<std::string>(BEHAVIOR_IMPORTROSPARAMETER_PARAM_OUTPUT_KEY, lValue);
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}