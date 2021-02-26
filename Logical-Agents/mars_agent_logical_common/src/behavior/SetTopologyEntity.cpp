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