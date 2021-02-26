#include "mars_agent_logical_common/behavior/OrderStatus/SetOrderState.h"

static const std::string BEHAVIOR_ORDER_STATE_READ_PARAM_NAME =
    "input_order_state";
static const std::string BEHAVIOR_ORDER_STATE_READ_PARAMETER_DESCRIPTION = "";

static const std::string BEHAVIOR_ORDER_STATE_WRITE_PARAM_NAME = "order_state";
static const std::string BEHAVIOR_ORDER_STATE_WRITE_PARAMETER_DESCRIPTION = "";

mars::agent::logical::common::behavior::SetOrderState::SetOrderState(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
  this->mStateEnums.insert(

      {{"ORDER_STATE_UNKNOWN",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_UNKNOWN},
       // Move order
       {"ORDER_STATE_MO_MOVE_ORDER_UNAVAILABLE",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_MO_MOVE_ORDER_UNAVAILABLE},
       {"ORDER_STATE_MO_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_MO_MOVE_ORDER_START},
       {"ORDER_STATE_MO_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_MO_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_MO_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_MO_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_MO_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_MO_MOVE_ORDER_ERROR},
       // Transport order step
       {"ORDER_STATE_TOS_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_MOVE_ORDER_START},
       {"ORDER_STATE_TOS_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TOS_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_TOS_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TOS_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_TOS_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_MOVE_ORDER_ERROR},
       {"ORDER_STATE_TOS_ACTION_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_START},
       {"ORDER_STATE_TOS_ACTION_ONGOING",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_ONGOING},
       {"ORDER_STATE_TOS_ACTION_FINISHED",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_FINISHED},
       {"ORDER_STATE_TOS_ACTION_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TOS_ACTION_ERROR},
       // Transport order
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_START},
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_TO_LOAD_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_MOVE_ORDER_ERROR},
       {"ORDER_STATE_TO_LOAD_ACTION_START",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TO_LOAD_ACTION_START},
       {"ORDER_STATE_TO_LOAD_ACTION_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_ACTION_ONGOING},
       {"ORDER_STATE_TO_LOAD_ACTION_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_LOAD_ACTION_FINISHED},
       {"ORDER_STATE_TO_LOAD_ACTION_ERROR",
        mars_agent_logical_msgs::OrderStatus::ORDER_STATE_TO_LOAD_ACTION_ERROR},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_START",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_START},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ONGOING},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_FINISHED},
       {"ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ERROR",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_MOVE_ORDER_ERROR},
       {"ORDER_STATE_TO_UNLOAD_ACTION_START",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_START},
       {"ORDER_STATE_TO_UNLOAD_ACTION_ONGOING",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_ONGOING},
       {"ORDER_STATE_TO_UNLOAD_ACTION_FINISHED",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_FINISHED},
       {"ORDER_STATE_TO_UNLOAD_ACTION_ERROR",
        mars_agent_logical_msgs::OrderStatus::
            ORDER_STATE_TO_UNLOAD_ACTION_ERROR}

      });
}

BT::PortsList
mars::agent::logical::common::behavior::SetOrderState::providedPorts()
{
  return {// Order status
          BT::InputPort<std::string>(
              BEHAVIOR_ORDER_STATE_READ_PARAM_NAME,
              BEHAVIOR_ORDER_STATE_READ_PARAMETER_DESCRIPTION),
          BT::OutputPort<unsigned int>(
              BEHAVIOR_ORDER_STATE_WRITE_PARAM_NAME,
              BEHAVIOR_ORDER_STATE_WRITE_PARAMETER_DESCRIPTION)};
}

BT::NodeStatus mars::agent::logical::common::behavior::SetOrderState::tick()
{
  BT::Result writeBackResult;
  BT::Optional<std::string> readResult;
  std::string inOrderState;
  unsigned int outOrderState;

  // get path id
  readResult = getInput<std::string>(BEHAVIOR_ORDER_STATE_READ_PARAM_NAME);
  if (!readResult)
  {
    // could not read the order status, which should be set
    MARS_LOG_ERROR(readResult.error());
    return BT::NodeStatus::FAILURE;
  }
  inOrderState = readResult.value();

  outOrderState = this->mStateEnums[inOrderState];

  // update order status on blackboard
  writeBackResult = setOutput<unsigned int>(
      BEHAVIOR_ORDER_STATE_WRITE_PARAM_NAME, outOrderState);

  // log error from setting output
  if (!writeBackResult)
  {
    MARS_LOG_ERROR(writeBackResult.error());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
