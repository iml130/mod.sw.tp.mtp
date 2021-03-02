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

#ifndef WAITFORNSECONDS_H
#define WAITFORNSECONDS_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/time.h>

#include <mars_common/Logger.h>
#include <mars_common/exception/ReadParamException.h>

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
class WaitForNSeconds : public BT::CoroActionNode
{
public:
  /**
   * @brief WaitForNSeconds Constructs an instance of this class.
   * @param name The name of this node.
   * @param config The configuration of this node. Gets forwarded to the super
   * class.
   */
  WaitForNSeconds(const std::string& name, const BT::NodeConfiguration& config);

  /**
   * @brief tick Implements the logic of this node.
   * @return Succesful, if the task is completed.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief providedPorts Defines the ports of this node.
   * @return List of ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief halt
   */
  void halt() override;

private:
  ros::Duration getWaitDuration();

  void writeFlag(const bool pFlagValue);
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // WAITFORNSECONDS_H
