#include "mars_agent_logical_common/behavior/Conditions/IsMoveOrder.h"

static const std::string BEHAVIOR_ISMOVEORDER_PARAM_NAME_ORDER =
    "order";
static const std::string BEHAVIOR_ISMOVEORDER_PARAM_DESCRIPTION_ORDER =
    "Order which should be tested, if it is a move order.";

mars::agent::logical::common::behavior::IsMoveOrder::IsMoveOrder(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::ConditionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::IsMoveOrder::providedPorts()
{
  return {// Order which shall be tested on being a move order
          BT::InputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
              BEHAVIOR_ISMOVEORDER_PARAM_NAME_ORDER,
              BEHAVIOR_ISMOVEORDER_PARAM_DESCRIPTION_ORDER)};
}

BT::NodeStatus mars::agent::logical::common::behavior::IsMoveOrder::tick()
{
  std::shared_ptr<mars::agent::logical::common::MoveOrder> moveOrder;
  BT::Optional<std::shared_ptr<mars::agent::logical::common::Order>> order;

  order = this->getInput<std::shared_ptr<mars::agent::logical::common::Order>>(
      BEHAVIOR_ISMOVEORDER_PARAM_NAME_ORDER);
  if (!order)
  {
    MARS_LOG_ERROR(order.error());
    return BT::NodeStatus::FAILURE;
  }

  moveOrder =
      std::dynamic_pointer_cast<mars::agent::logical::common::MoveOrder>(
          order.value());
  if (!moveOrder)
  {
    // its not a move order
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
