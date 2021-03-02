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

#include "mars_agent_logical_common/behavior/SetId.h"
#include "mars_common/Logger.h"

BT::NodeStatus mars::agent::logical::common::behavior::SetId::tick()
{
  BT::Optional<std::string> lUUIDString;
  BT::Result lResult;

  lUUIDString =
      this->getInput<std::string>(BEHAVIOR_SETID_PARAM_NAME_UUID_STRING);
  if (!lUUIDString)
  {
    MARS_LOG_ERROR(lUUIDString.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<mars::common::Id>(BEHAVIOR_SETID_PARAM_NAME_ID, mars::common::Id(lUUIDString.value()));
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}