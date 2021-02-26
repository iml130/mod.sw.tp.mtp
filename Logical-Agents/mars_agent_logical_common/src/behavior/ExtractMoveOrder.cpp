#include "mars_agent_logical_common/behavior/ExtractMoveOrder.h"

static const std::string BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_MOVE_ORDER =
    "order";
static const std::string BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_ORDER_ID =
    "order_id";
static const std::string BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_DESTINATION =
    "destination";
static const std::string
    BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_DESTINATION_RESERVATION_DURATION =
        "destination_reservation_duration";

mars::agent::logical::common::behavior::ExtractMoveOrder::ExtractMoveOrder(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::ExtractMoveOrder::providedPorts()
{
  return {
      BT::InputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
          BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_MOVE_ORDER),
      BT::OutputPort<mars::common::Id>(
          BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_ORDER_ID),
      BT::OutputPort<std::shared_ptr<mars::routing::common::topology::Entity>>(
          BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_DESTINATION),
      BT::OutputPort<ros::Duration>(
          BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_DESTINATION_RESERVATION_DURATION)};
}

BT::NodeStatus mars::agent::logical::common::behavior::ExtractMoveOrder::tick()
{
  std::shared_ptr<mars::agent::logical::common::MoveOrder> lMoveOrder;

  BT::Optional<std::shared_ptr<mars::agent::logical::common::Order>> lOrder;
  BT::Result lResult;

  lOrder = this->getInput<std::shared_ptr<mars::agent::logical::common::Order>>(
      BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_MOVE_ORDER);
  if (!lOrder)
  {
    MARS_LOG_ERROR(lOrder.error());
    return BT::NodeStatus::FAILURE;
  }

  lMoveOrder =
      std::dynamic_pointer_cast<mars::agent::logical::common::MoveOrder>(
          lOrder.value());
  if (!lMoveOrder)
  {
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<mars::common::Id>(
      BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_ORDER_ID, lMoveOrder->getId());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  if (lMoveOrder->getDestination()->getType() >=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      lMoveOrder->getDestination()->getType() <=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
  {
    lResult = this->setOutput<
        std::shared_ptr<mars::routing::common::topology::Entity>>(
        BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_DESTINATION,
        std::make_shared<mars::routing::common::topology::Vertex>(
            lMoveOrder->getDestination()->getId()));
  }
  else if (lMoveOrder->getDestination()->getType() >=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
           lMoveOrder->getDestination()->getType() <=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX)
  {
    lResult = this->setOutput<
        std::shared_ptr<mars::routing::common::topology::Entity>>(
        BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_DESTINATION,
        std::make_shared<mars::routing::common::topology::Edge>(
            lMoveOrder->getDestination()->getId()));
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<ros::Duration>(
      BEHAVIOR_EXTRACTMOVEORDER_PARAM_NAME_DESTINATION_RESERVATION_DURATION,
      lMoveOrder->getDestinationReservationDuration());

  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

