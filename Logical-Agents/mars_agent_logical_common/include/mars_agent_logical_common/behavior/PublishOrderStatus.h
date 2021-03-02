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

#ifndef PUBLISHORDERSTATUS_H
#define PUBLISHORDERSTATUS_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <mars_agent_logical_msgs/OrderStatus.h>
#include <mars_common/Id.h>
#include <mars_common/Logger.h>

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
class PublishOrderStatus : public BT::SyncActionNode
{
public:
  PublishOrderStatus(const std::string& pName,
                     const BT::NodeConfiguration& pConfig);

  /**
   * @brief providedPorts
   * @return
   */
  static BT::PortsList providedPorts();

  /**
   * @brief tick Is called by every tree tick.
   * @return
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief getParams Gets necessary parameter from the blackboard
   * @return True, if all parameter could be read.  False, otherwise.
   */
  bool getParams();

  ros::Publisher mOrderStatusPub;

  unsigned int mCurrentOrderStatus;
  unsigned int mCurrentOrderType;
  unsigned int mCurrentOrderState;
  mars::common::Id mCurrentOrderId;

};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // PUBLISHORDERSTATUS_H
