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

#include "mars_agent_logical_common/behavior/SetDuration.h"

static const std::string BEHAVIOR_SETDURATION_PARAM_NAME_RESERVATION_DURATION =
    "reservation_duration";
static const std::string BEHAVIOR_SETDURATION_PARAM_NAME_DURATION = "duration";

mars::agent::logical::common::behavior::SetDuration::SetDuration(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::SetDuration::providedPorts()
{
  {
    return {
        BT::InputPort< double>(BEHAVIOR_SETDURATION_PARAM_NAME_DURATION),
        BT::OutputPort<ros::Duration>(
            BEHAVIOR_SETDURATION_PARAM_NAME_RESERVATION_DURATION)};
  }
}

BT::NodeStatus mars::agent::logical::common::behavior::SetDuration::tick()
{
  BT::Optional< double> duration;
  BT::Result lResult;

  duration =
      this->getInput< double>(BEHAVIOR_SETDURATION_PARAM_NAME_DURATION);
  if (!duration)
  {
    MARS_LOG_ERROR(duration.error());
  }

  lResult = this->setOutput<ros::Duration>(
      BEHAVIOR_SETDURATION_PARAM_NAME_RESERVATION_DURATION,
      ros::Duration(duration.value()));
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
