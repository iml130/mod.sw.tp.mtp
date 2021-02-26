#include "mars_agent_logical_common/behavior/OrderStatus/SetOrderType.h"

static const std::string BEHAVIOR_ORDER_TYPE_READ_PARAM_NAME =
    "input_order_type";
static const std::string BEHAVIOR_ORDER_TYPE_READ_PARAMETER_DESCRIPTION = "";

static const std::string BEHAVIOR_ORDER_TYPE_WRITE_PARAM_NAME = "order_type";
static const std::string BEHAVIOR_ORDER_TYPE_WRITE_PARAMETER_DESCRIPTION = "";

mars::agent::logical::common::behavior::SetOrderType::SetOrderType(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
  this->mTypeEnums.insert(
      {{"ORDER_TYPE_UNKNOWN",
        mars_agent_logical_msgs::OrderStatus::ORDER_TYPE_UNKNOWN},
       {"ORDER_TYPE_MOVE_ORDER",
        mars_agent_logical_msgs::OrderStatus::ORDER_TYPE_MOVE_ORDER},
       {"ORDER_TYPE_TRANSPORT_ORDER_STEP",
        mars_agent_logical_msgs::OrderStatus::ORDER_TYPE_TRANSPORT_ORDER_STEP},
       {"ORDER_TYPE_TRANSPORT_ORDER",
        mars_agent_logical_msgs::OrderStatus::ORDER_TYPE_TRANSPORT_ORDER}});
}

BT::PortsList
mars::agent::logical::common::behavior::SetOrderType::providedPorts()
{
  return {// Order status
          BT::InputPort<std::string>(
              BEHAVIOR_ORDER_TYPE_READ_PARAM_NAME,
              BEHAVIOR_ORDER_TYPE_READ_PARAMETER_DESCRIPTION),
          BT::OutputPort<unsigned int>(
              BEHAVIOR_ORDER_TYPE_WRITE_PARAM_NAME,
              BEHAVIOR_ORDER_TYPE_WRITE_PARAMETER_DESCRIPTION)};
}

BT::NodeStatus mars::agent::logical::common::behavior::SetOrderType::tick()
{
  BT::Result writeBackResult;
  BT::Optional<std::string> readResult;
  std::string inOrderType;
  unsigned int outOrderType;

  // get path id
  readResult = getInput<std::string>(BEHAVIOR_ORDER_TYPE_READ_PARAM_NAME);
  if (!readResult)
  {
    // could not read the order status, which should be set
    MARS_LOG_ERROR(readResult.error());
    return BT::NodeStatus::FAILURE;
  }
  inOrderType = readResult.value();

  outOrderType = this->mTypeEnums[inOrderType];

  // update order status on blackboard
  writeBackResult = setOutput<unsigned int>(
      BEHAVIOR_ORDER_TYPE_WRITE_PARAM_NAME, outOrderType);

  // log error from setting output
  if (!writeBackResult)
  {
    MARS_LOG_ERROR(writeBackResult.error());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
