//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

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

