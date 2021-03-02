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

#include "mars_agent_logical_common/behavior/WaitForNSeconds.h"

static const std::string PARAM_NAME_WAIT_DURATION = "wait_duration";
static const std::string PARAM_DESCRIPTION_WAIT_DURATION =
    "Duration to wait until this node returns success.";

mars::agent::logical::common::behavior::WaitForNSeconds::WaitForNSeconds(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::CoroActionNode(name, config)
{
}

BT::NodeStatus mars::agent::logical::common::behavior::WaitForNSeconds::tick()
{
  // vom blackboard lesen oder aus ros launch ?
  ros::Duration waitDuration = getWaitDuration();

  ros::Time startTime = ros::Time::now();

  while ((ros::Time::now() - startTime) <= waitDuration)
  {
    this->setStatusRunningAndYield();
  }

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList
mars::agent::logical::common::behavior::WaitForNSeconds::providedPorts()
{
  return {BT::InputPort<double>(PARAM_NAME_WAIT_DURATION,
                                PARAM_DESCRIPTION_WAIT_DURATION)};
}

void mars::agent::logical::common::behavior::WaitForNSeconds::halt()
{
  std::cout << name() << ": Halted." << std::endl;

  // Do not forget to call this at the end.
  CoroActionNode::halt();
}

ros::Duration
mars::agent::logical::common::behavior::WaitForNSeconds::getWaitDuration()
{
  BT::Optional<double> waitDuration;

  waitDuration = this->getInput<double>(PARAM_NAME_WAIT_DURATION);
  if (!waitDuration)
  {
    MARS_LOG_ERROR(waitDuration.error());
    throw mars::common::exception::ReadParamException(
        "Could not read the waiting duration from the blackboard.");
  }
  else
  {
    return ros::Duration(waitDuration.value());
  }
}
