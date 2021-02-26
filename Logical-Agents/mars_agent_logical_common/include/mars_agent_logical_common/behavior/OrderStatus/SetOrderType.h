#ifndef SETORDERTYPE_H
#define SETORDERTYPE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <mars_agent_logical_msgs/OrderStatus.h>
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
class SetOrderType :public BT::SyncActionNode
{
public:
  SetOrderType(const std::string& pName,
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

private:
  std::unordered_map<std::string, unsigned int> mTypeEnums;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // SETORDERTYPE_H
