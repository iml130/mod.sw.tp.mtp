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

#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTTIME_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTTIME_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <ros/ros.h>

static const std::string BEHAVIOR_SETCURRENTTIME_PARAM_NAME_TIME = "time";

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
namespace behavior
{
class SetCurrentTime : public BT::SyncActionNode
{
public:
  SetCurrentTime(const std::string& pName, const BT::NodeConfiguration& pConfig)
      : BT::SyncActionNode(pName, pConfig)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<ros::Time>(BEHAVIOR_SETCURRENTTIME_PARAM_NAME_TIME)};
  }

  BT::NodeStatus tick() override;

private:
  ros::NodeHandle mNodeHandle;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTTIME_H
