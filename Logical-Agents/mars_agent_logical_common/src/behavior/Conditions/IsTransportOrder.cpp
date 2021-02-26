#include "mars_agent_logical_common/behavior/Conditions/IsTransportOrder.h"

static const std::string BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_NAME_ORDER =
    "order";
static const std::string BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_DESCRIPTION_ORDER =
    "Order which should be tested, if it is a transport order.";

mars::agent::logical::common::behavior::IsTransportOrder::IsTransportOrder(const std::string &pName, const BT::NodeConfiguration &pConfig): BT::ConditionNode(pName, pConfig)
{

}

BT::PortsList mars::agent::logical::common::behavior::IsTransportOrder::providedPorts()
{
  return {// Order which shall be tested on being a transport order
          BT::InputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
              BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_NAME_ORDER,
              BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_DESCRIPTION_ORDER)};
}

BT::NodeStatus mars::agent::logical::common::behavior::IsTransportOrder::tick()
{
  std::shared_ptr<mars::agent::logical::common::TransportOrder> transportOrder;
  BT::Optional<std::shared_ptr<mars::agent::logical::common::Order>> order;

  order = this->getInput<std::shared_ptr<mars::agent::logical::common::Order>>(
      BEHAVIOR_IS_TRANSPORT_ORDER_PARAM_NAME_ORDER);
  if (!order)
  {
    MARS_LOG_ERROR(order.error());
    return BT::NodeStatus::FAILURE;
  }

  transportOrder =
      std::dynamic_pointer_cast<mars::agent::logical::common::TransportOrder>(
          order.value());
  if (!transportOrder)
  {
    // its not a move order
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
