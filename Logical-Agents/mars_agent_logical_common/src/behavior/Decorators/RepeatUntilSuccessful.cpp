#include "mars_agent_logical_common/behavior/Decorators/RepeatUntilSuccessful.h"

mars::agent::logical::common::behavior::RepeatUntilSuccessful::
    RepeatUntilSuccessful(const std::string& pName)
    : BT::DecoratorNode(pName, {})
{
  setRegistrationID("RepeatOnce");
}

void mars::agent::logical::common::behavior::RepeatUntilSuccessful::halt()
{
  BT::DecoratorNode::halt();
}

BT::NodeStatus mars::agent::logical::common::behavior::RepeatUntilSuccessful::tick()
{
  BT::NodeStatus child_state = this->child_node_->executeTick();
  BT::NodeStatus nodeStatus;

  switch (child_state)
  {
  case BT::NodeStatus::SUCCESS:
    nodeStatus = BT::NodeStatus::SUCCESS;
    break;

  case BT::NodeStatus::FAILURE:
    nodeStatus = BT::NodeStatus::RUNNING;
    break;

  case BT::NodeStatus::RUNNING:
    nodeStatus = BT::NodeStatus::RUNNING;
    break;

  default:
    throw BT::LogicError("A child node must never return IDLE");
  }
  return nodeStatus;
}
