#ifndef SETDURATION_H
#define SETDURATION_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/time.h>

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
class SetDuration : public BT::SyncActionNode
{
public:
  SetDuration(const std::string& pName, const BT::NodeConfiguration& pConfig);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // SETDURATION_H
