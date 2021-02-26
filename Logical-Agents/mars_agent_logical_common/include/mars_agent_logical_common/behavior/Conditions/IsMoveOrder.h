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
