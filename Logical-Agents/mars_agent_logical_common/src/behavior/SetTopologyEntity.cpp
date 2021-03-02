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

#include "mars_agent_logical_common/behavior/SetTopologyEntity.h"
#include "mars_common/Logger.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

BT::NodeStatus mars::agent::logical::common::behavior::SetTopologyEntity::tick()
{
  BT::Optional<mars::common::Id> lId;
  BT::Optional<std::string> lTypeString;
  BT::Result lResult;

  int lType;

  lId = this->getInput<mars::common::Id>(BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_ID);
  if (!lId)
  {
    MARS_LOG_ERROR(lId.error());
    return BT::NodeStatus::FAILURE;
  }

  lTypeString = this->getInput<std::string>(BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_TYPE);
  if (!lTypeString)
  {
    MARS_LOG_ERROR(lTypeString.error());
    return BT::NodeStatus::FAILURE;
  }

  lType = BT::convertFromString<int>(lTypeString.value());

  if (lType >= mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      lType <= mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
  {
    lResult = this->setOutput<std::shared_ptr<mars::routing::common::topology::Entity>>(
        BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_ENTITY,
        std::make_shared<mars::routing::common::topology::Vertex>(lId.value()));
  }
  else if (lType >= mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
           lType <= mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX)
  {
    lResult = this->setOutput<std::shared_ptr<mars::routing::common::topology::Entity>>(
        BEHAVIOR_SETTOPOLOGYENTITY_PARAM_NAME_ENTITY,
        std::make_shared<mars::routing::common::topology::Edge>(lId.value()));
  }
  else
  {
    MARS_LOG_WARN_UNKNOWN_TOPOLOGY_ENTITY_TYPE();
  }

  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}