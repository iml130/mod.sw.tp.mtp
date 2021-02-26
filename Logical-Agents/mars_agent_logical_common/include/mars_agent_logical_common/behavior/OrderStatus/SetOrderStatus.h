#ifndef SETORDERSTATUS_H
#define SETORDERSTATUS_H

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
class SetOrderStatus : public BT::SyncActionNode
{
public:
  SetOrderStatus(const std::string& pName,
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
  std::unordered_map<std::string, unsigned int> mStatusEnums;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // SETORDERSTATUS_H
