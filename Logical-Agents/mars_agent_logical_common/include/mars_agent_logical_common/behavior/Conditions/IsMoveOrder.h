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

#ifndef ISMOVEORDER_H
#define ISMOVEORDER_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "mars_agent_logical_common/MoveOrder.h"
#include "mars_agent_logical_common/Order.h"

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

class IsMoveOrder :public BT::ConditionNode
{
public:
  IsMoveOrder(const std::string& pName,
              const BT::NodeConfiguration& pConfig);

  /**
   * @brief providedPorts
   * @return
   */
  static BT::PortsList providedPorts();

  /**
   * @brief tick
   * @return
   */
  BT::NodeStatus tick() override;
};

} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // ISMOVEORDER_H
