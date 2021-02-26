#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETID_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETID_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "mars_common/Id.h"

static const std::string BEHAVIOR_SETID_PARAM_NAME_ID = "id";
static const std::string BEHAVIOR_SETID_PARAM_NAME_UUID_STRING = "uuid_string";

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
class SetId : public BT::SyncActionNode
{
public:
  SetId(const std::string& pName, const BT::NodeConfiguration& pConfig)
      : BT::SyncActionNode(pName, pConfig)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(BEHAVIOR_SETID_PARAM_NAME_UUID_STRING),
            BT::OutputPort<mars::common::Id>(BEHAVIOR_SETID_PARAM_NAME_ID)};
  }

  BT::NodeStatus tick() override;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETID_H
