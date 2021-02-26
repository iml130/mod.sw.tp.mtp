#ifndef SETORDERSTATE_H
#define SETORDERSTATE_H

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
class SetOrderState : public BT::SyncActionNode
{
public:
  SetOrderState(const std::string& pName,
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
  std::unordered_map<std::string, unsigned int> mStateEnums;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // SETORDERSTATE_H
