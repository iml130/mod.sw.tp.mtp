#ifndef ISTRANSPORTORDER_H
#define ISTRANSPORTORDER_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "mars_agent_logical_common/TransportOrder.h"
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
class IsTransportOrder : public BT::ConditionNode
{
public:
  IsTransportOrder(const std::string& pName,
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
#endif // ISTRANSPORTORDER_H
