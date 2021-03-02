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

#include "mars_agent_logical_common/behavior/StringToBool.h"

BT::NodeStatus mars::agent::logical::common::behavior::StringToBool::tick()
{
  BT::Optional<std::string> lBoolString;
  BT::Result lResult;

  lBoolString = this->getInput<std::string>(BEHAVIOR_STRINGTOBOOL_PARAM_NAME_STRING);
  if (!lBoolString)
  {
    MARS_LOG_ERROR(lBoolString.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<bool>(BEHAVIOR_STRINGTOBOOL_PARAM_NAME_BOOL,
                                  BT::convertFromString<bool>(lBoolString.value()));
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}