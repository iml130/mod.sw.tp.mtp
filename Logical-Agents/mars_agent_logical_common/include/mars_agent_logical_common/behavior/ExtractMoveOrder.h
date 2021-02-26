#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_EXTRACTMOVEORDER_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_EXTRACTMOVEORDER_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "mars_agent_logical_common/Order.h"
#include "mars_routing_common/topology/Entity.h"

#include "mars_agent_logical_common/MoveOrder.h"
#include "mars_common/Logger.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

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
class ExtractMoveOrder : public BT::SyncActionNode
{
public:
  ExtractMoveOrder(const std::string& pName, const BT::NodeConfiguration& pConfig);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_EXTRACTMOVEORDER_H
