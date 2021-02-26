#include "mars_agent_logical_common/behavior/Decorators/RepeatUntilDone.h"

static const std::string REPEAT_UNTIL_DONE_PARAM_NAME_STATUS = "done_status";
static const std::string REPEAT_UNTIL_DONE_PARAMETER_DESCRIPTION_STATUS =
    "done_status";

mars::agent::logical::common::behavior::RepeatUntilDone::RepeatUntilDone(
    const std::string& pName)
    : BT::DecoratorNode(pName, {})
{
  setRegistrationID("RepeatUntilDone");
}

mars::agent::logical::common::behavior::RepeatUntilDone::RepeatUntilDone(
    const std::string& name, const BT::NodeConfiguration& config)
    : DecoratorNode(name, config)
{
}

BT::PortsList
mars::agent::logical::common::behavior::RepeatUntilDone::providedPorts()
{
  return {BT::InputPort<bool>(REPEAT_UNTIL_DONE_PARAM_NAME_STATUS,
                              REPEAT_UNTIL_DONE_PARAMETER_DESCRIPTION_STATUS)};
}

BT::NodeStatus mars::agent::logical::common::behavior::RepeatUntilDone::tick()
{
  BT::NodeStatus child_state = this->child_node_->executeTick();
  BT::NodeStatus nodeStatus;

  switch (child_state)
  {
  case BT::NodeStatus::SUCCESS:
    if (readDoneStatus())
    {
      nodeStatus = BT::NodeStatus::SUCCESS;
    }
    else
    {
      nodeStatus = BT::NodeStatus::RUNNING;
    }
    break;

  case BT::NodeStatus::FAILURE:
    nodeStatus = (BT::NodeStatus::FAILURE);
    break;

  case BT::NodeStatus::RUNNING:
    nodeStatus = BT::NodeStatus::RUNNING;
    break;

  default:
    throw BT::LogicError("A child node must never return IDLE");
  }
  return nodeStatus;
}

const bool
mars::agent::logical::common::behavior::RepeatUntilDone::readDoneStatus()
{
  BT::Optional<bool> getDoneStatusResult;

  getDoneStatusResult = getInput<bool>(REPEAT_UNTIL_DONE_PARAM_NAME_STATUS);
  if (!getDoneStatusResult)
  {
    throw mars::common::exception::ReadParamException(
        "Could not read the number of current allocated entities on port: " +
        REPEAT_UNTIL_DONE_PARAM_NAME_STATUS + "\n");
  }
  return getDoneStatusResult.value();
}

void mars::agent::logical::common::behavior::RepeatUntilDone::halt()
{
  BT::DecoratorNode::halt();
}
