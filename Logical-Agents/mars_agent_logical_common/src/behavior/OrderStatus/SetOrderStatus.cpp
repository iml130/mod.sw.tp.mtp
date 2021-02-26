#include "mars_agent_logical_common/behavior/OrderStatus/SetOrderStatus.h"

static const std::string BEHAVIOR_ORDER_STATUS_READ_PARAM_NAME =
    "input_order_status";
static const std::string BEHAVIOR_ORDER_STATUS_READ_PARAMETER_DESCRIPTION = "";

static const std::string BEHAVIOR_ORDER_STATUS_WRITE_PARAM_NAME =
    "order_status";
static const std::string BEHAVIOR_ORDER_STATUS_WRITE_PARAMETER_DESCRIPTION = "";

mars::agent::logical::common::behavior::SetOrderStatus::SetOrderStatus(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{

  this->mStatusEnums.insert(
      {{"ORDER_STATUS_UNKNOWN",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_UNKNOWN},
       {"ORDER_STATUS_STARTED",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_STARTED},
       {"ORDER_STATUS_ONGOING",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_ONGOING},
       {"ORDER_STATUS_FINISHED",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_FINISHED},
       {"ORDER_STATUS_WAITING",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_WAITING},
       {"ORDER_STATUS_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_ERROR}});
}

BT::PortsList
mars::agent::logical::common::behavior::SetOrderStatus::providedPorts()
{
  return {// Order status
          BT::InputPort<std::string>(
              BEHAVIOR_ORDER_STATUS_READ_PARAM_NAME,
              BEHAVIOR_ORDER_STATUS_READ_PARAMETER_DESCRIPTION),
          BT::OutputPort<unsigned int>(
              BEHAVIOR_ORDER_STATUS_WRITE_PARAM_NAME,
              BEHAVIOR_ORDER_STATUS_WRITE_PARAMETER_DESCRIPTION)};
}

BT::NodeStatus mars::agent::logical::common::behavior::SetOrderStatus::tick()
{
  BT::Result writeBackResult;
  BT::Optional<std::string> readResult;
  std::string inOrderStatus;
  unsigned int outOrderStatus;

  // get path id
  readResult = getInput<std::string>(BEHAVIOR_ORDER_STATUS_READ_PARAM_NAME);
  if (!readResult)
  {
    // could not read the order status, which should be set
    MARS_LOG_ERROR(readResult.error());
    return BT::NodeStatus::FAILURE;
  }
  inOrderStatus = readResult.value();

  outOrderStatus = this->mStatusEnums[inOrderStatus];

  // update order status on blackboard
  writeBackResult = setOutput<unsigned int>(
      BEHAVIOR_ORDER_STATUS_WRITE_PARAM_NAME, outOrderStatus);

  // log error from setting output
  if (!writeBackResult)
  {
    MARS_LOG_ERROR(writeBackResult.error());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
