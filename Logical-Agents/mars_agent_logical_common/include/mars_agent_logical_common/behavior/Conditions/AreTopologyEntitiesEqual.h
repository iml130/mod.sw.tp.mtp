#ifndef ARETOPOLOGYENTITIESEQUAL_H
#define ARETOPOLOGYENTITIESEQUAL_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <mars_routing_common/topology/Entity.h>
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
class AreTopologyEntitiesEqual : public BT::ConditionNode
{
public:
  AreTopologyEntitiesEqual(const std::string& pName,
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
#endif // ARETOPOLOGYENTITIESEQUAL_H
