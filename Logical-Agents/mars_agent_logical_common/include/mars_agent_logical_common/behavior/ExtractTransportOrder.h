#ifndef EXTRACTTRANSPORTORDER_H
#define EXTRACTTRANSPORTORDER_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "mars_agent_logical_common/Order.h"
#include "mars_routing_common/topology/Entity.h"

#include "mars_agent_logical_common/TransportOrder.h"
#include "mars_agent_logical_common/TransportOrderStep.h"
#include "mars_common/Logger.h"

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
class ExtractTransportOrder : public BT::SyncActionNode
{
public:
  ExtractTransportOrder(const std::string& pName, const BT::NodeConfiguration& pConfig);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // EXTRACTTRANSPORTORDER_H
