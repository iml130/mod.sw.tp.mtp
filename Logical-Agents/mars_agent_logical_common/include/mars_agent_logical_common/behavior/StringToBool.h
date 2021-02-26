#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_STRINGTOBOOL_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_STRINGTOBOOL_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include "mars_common/Logger.h"

static const std::string BEHAVIOR_STRINGTOBOOL_PARAM_NAME_BOOL = "output";
static const std::string BEHAVIOR_STRINGTOBOOL_PARAM_NAME_STRING = "input";

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
class StringToBool : public BT::SyncActionNode
{
public:
  StringToBool(const std::string& pName, const BT::NodeConfiguration& pConfig)
      : BT::SyncActionNode(pName, pConfig)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(BEHAVIOR_STRINGTOBOOL_PARAM_NAME_STRING),
            BT::OutputPort<bool>(BEHAVIOR_STRINGTOBOOL_PARAM_NAME_BOOL)};
  }

  BT::NodeStatus tick() override;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_STRINGTOBOOL_H
