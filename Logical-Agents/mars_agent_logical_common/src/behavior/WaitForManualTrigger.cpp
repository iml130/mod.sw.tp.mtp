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

#include "mars_agent_logical_common/behavior/WaitForManualTrigger.h"

static const std::string SERVICE_ERROR_MSGS_NOT_TRIGGER_EXPECTED =
    "Received a trigger while not expecting one, it will be ignored.";

mars::agent::logical::common::behavior::WaitForManualTrigger::
    WaitForManualTrigger(const std::string& name,
                         const BT::NodeConfiguration& config)
    : BT::CoroActionNode(name, config)
{
  mWaitForTrigger = false;
  mTriggerReceived = false;
}

BT::NodeStatus
mars::agent::logical::common::behavior::WaitForManualTrigger::tick()
{
  this->mWaitForTrigger = true;

  while (!this->mTriggerReceived.load())
  {
    this->setStatusRunningAndYield();
  }

  // reset flags
  this->mWaitForTrigger = false;
  this->mTriggerReceived = false;

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList
mars::agent::logical::common::behavior::WaitForManualTrigger::providedPorts()
{
  // no ports provided
  return {};
}

bool mars::agent::logical::common::behavior::WaitForManualTrigger::
    serviceTriggerCallback(
        mars_agent_logical_srvs::ManualActionDoneRequest& pReq,
        mars_agent_logical_srvs::ManualActionDoneResponse& pRes)
{
  if (mWaitForTrigger.load())
  {
    mTriggerReceived = true;
  }
  else
  {
    pRes.result.result = mars_common_msgs::Result::RESULT_ERROR;
    pRes.result.description = SERVICE_ERROR_MSGS_NOT_TRIGGER_EXPECTED;
  }
  return true;
}

std::atomic<bool> mars::agent::logical::common::behavior::WaitForManualTrigger::
    mTriggerReceived;

std::atomic<bool> mars::agent::logical::common::behavior::WaitForManualTrigger::
    mWaitForTrigger;
