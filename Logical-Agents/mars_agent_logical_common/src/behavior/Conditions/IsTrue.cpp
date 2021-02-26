#include "mars_agent_logical_common/behavior/Conditions/IsTrue.h"

static const std::string PARAM_NAME_FLAG = "input";

mars::agent::logical::common::behavior::IsTrue::IsTrue(const std::string& pName,
                                                       const BT::NodeConfiguration& pConfig)
    : BT::ConditionNode(pName, pConfig)
{
}

BT::PortsList mars::agent::logical::common::behavior::IsTrue::providedPorts()
{
  return {BT::InputPort<bool>(PARAM_NAME_FLAG)};
}

BT::NodeStatus mars::agent::logical::common::behavior::IsTrue::tick()
{
  BT::Optional<bool> lFlag = this->getInput<bool>(PARAM_NAME_FLAG);

  if (!lFlag)
  {
    MARS_LOG_ERROR(lFlag.error());
    return BT::NodeStatus::FAILURE;
  }

  return (lFlag.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
