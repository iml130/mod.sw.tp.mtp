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

#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETTOPOLOGYENTITY_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETTOPOLOGYENTITY_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "mars_routing_common/topology/Entity.h"

static const std::string BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_ID = "entity_id";
static const std::string BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_TYPE = "entity_type";
static const std::string BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_ENTITY = "entity";

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
class SetTopologyEntity : public BT::SyncActionNode
{
public:
  SetTopologyEntity(const std::string& pName, const BT::NodeConfiguration& pConfig)
      : BT::SyncActionNode(pName, pConfig)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<mars::common::Id>(BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_ID),
            BT::InputPort<std::string>(BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_TYPE),
            BT::OutputPort<std::shared_ptr<mars::routing::common::topology::Entity>>(
                BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_ENTITY)};
  }

  BT::NodeStatus tick() override;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETTOPOLOGYENTITY_H
