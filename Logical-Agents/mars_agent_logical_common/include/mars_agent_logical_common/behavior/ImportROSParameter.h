#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_IMPORTROSPARAMETER_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_IMPORTROSPARAMETER_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <ros/ros.h>
#include <ros/service_client.h>

static const std::string BEHAVIOR_IMPORTROSPARAMETER_PARAM_NAME = "parameter";
static const std::string BEHAVIOR_IMPORTROSPARAMETER_PARAM_OUTPUT_KEY = "output_key";

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
class ImportROSParameter : public BT::SyncActionNode
{
public:
  ImportROSParameter(const std::string& pName, const BT::NodeConfiguration& pConfig)
      : BT::SyncActionNode(pName, pConfig), mNHPriv("~")
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(BEHAVIOR_IMPORTROSPARAMETER_PARAM_NAME),
            BT::OutputPort<std::string>(BEHAVIOR_IMPORTROSPARAMETER_PARAM_OUTPUT_KEY)};
  }

  BT::NodeStatus tick() override;

private:
  ros::NodeHandle mNHPriv;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_IMPORTROSPARAMETER_H
