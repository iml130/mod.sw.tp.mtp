#include "mars_agent_logical_common/behavior/Decorators/RepeatOncePerTick.h"

mars::agent::logical::common::behavior::RepeatOncePerTick::RepeatOncePerTick(
    const std::string& pName)
    : BT::DecoratorNode(pName, {})
{
  setRegistrationID("RepeatOnce");
}

BT::NodeStatus mars::agent::logical::common::behavior::RepeatOncePerTick::tick()
{
  BT::NodeStatus child_state = this->child_node_->executeTick();
  BT::NodeStatus nodeStatus;

  switch (child_state)
  {
  case BT::NodeStatus::SUCCESS:
    nodeStatus = BT::NodeStatus::RUNNING;
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

void mars::agent::logical::common::behavior::RepeatOncePerTick::halt()
{
  BT::DecoratorNode::halt();
}
