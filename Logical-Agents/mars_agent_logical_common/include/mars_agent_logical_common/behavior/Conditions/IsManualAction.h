#ifndef ISMANUALACTION_H
#define ISMANUALACTION_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <mars_common/Logger.h>
#include "mars_agent_logical_common/RobotAction.h"


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
class IsManualAction : public BT::ConditionNode
{
public:
  IsManualAction(const std::string& pName,
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
#endif // ISMANUALACTION_H
