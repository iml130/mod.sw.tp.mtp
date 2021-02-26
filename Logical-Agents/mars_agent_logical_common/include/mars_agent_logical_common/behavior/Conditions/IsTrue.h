#ifndef ISTRUE_H
#define ISTRUE_H

#include <behaviortree_cpp_v3/behavior_tree.h>
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
class IsTrue : public BT::ConditionNode
{
public:
  IsTrue(const std::string& pName, const BT::NodeConfiguration& pConfig);

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

#endif // ISTRUE_H
