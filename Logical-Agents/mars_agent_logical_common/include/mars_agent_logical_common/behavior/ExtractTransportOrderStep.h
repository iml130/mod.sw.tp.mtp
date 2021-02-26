#ifndef EXTRACTTRANSPORTORDERSTEP_H
#define EXTRACTTRANSPORTORDERSTEP_H

#include <behaviortree_cpp_v3/behavior_tree.h>

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
class ExtractTransportOrderStep : public BT::SyncActionNode
{
public:
  ExtractTransportOrderStep(const std::string& pName, const BT::NodeConfiguration& pConfig);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // EXTRACTTRANSPORTORDERSTEP_H
