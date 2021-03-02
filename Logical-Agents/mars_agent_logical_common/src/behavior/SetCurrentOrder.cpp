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

#include "mars_agent_logical_common/behavior/SetCurrentOrder.h"
#include "mars_agent_logical_common/MoveOrder.h"
#include "mars_common/Logger.h"

mars::agent::logical::common::behavior::SetCurrentOrder::SetCurrentOrder(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig), mNHPriv("~")
{
  ros::NodeHandle nh;

  this->mOrderStatusPub = nh.advertise<mars_agent_logical_msgs::OrderStatus>(
      "/order_status", 10, true);
}

BT::PortsList
mars::agent::logical::common::behavior::SetCurrentOrder::providedPorts()
{
  return {
      BT::InputPort<std::string>(
          BEHAVIOR_SETCURRENTORDER_PARAM_NAME_SERVICE_NAME_ADD_MOVE_ORDER),
      BT::InputPort<std::string>(
          BEHAVIOR_SETCURRENTORDER_PARAM_NAME_SERVICE_NAME_ADD_TRANSPORT_ORDER),
      BT::OutputPort<mars::common::Id>(
          BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER_ID),
      BT::OutputPort<std::shared_ptr<mars::agent::logical::common::Order>>(
          BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER),
      BT::OutputPort<bool>(BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER_FLAG)};
}

BT::NodeStatus mars::agent::logical::common::behavior::SetCurrentOrder::tick()
{
  BT::Result lResult;

  if (!this->mServiceAddMoveOrder || !this->mServiceAddTransportOrder)
  {
    BT::Optional<std::string> lServiceNameAddMoveOrder;
    BT::Optional<std::string> lServiceNameAddTransportOrder;

    lServiceNameAddMoveOrder = this->getInput<std::string>(
        BEHAVIOR_SETCURRENTORDER_PARAM_NAME_SERVICE_NAME_ADD_MOVE_ORDER);
    if (!lServiceNameAddMoveOrder)
    {
      MARS_LOG_WARN(lServiceNameAddMoveOrder.error());
      lServiceNameAddMoveOrder.value() =
          BEHAVIOR_SETCURRENTORDER_DEFAULT_SERVICE_NAME_ADD_MOVE_ORDER;
    }

    lServiceNameAddTransportOrder = this->getInput<std::string>(
        BEHAVIOR_SETCURRENTORDER_PARAM_NAME_SERVICE_NAME_ADD_TRANSPORT_ORDER);
    if (!lServiceNameAddTransportOrder)
    {
      MARS_LOG_WARN(lServiceNameAddTransportOrder.error());
      lServiceNameAddTransportOrder.value() =
          BEHAVIOR_SETCURRENTORDER_DEFAULT_SERVICE_NAME_ADD_TRANSPORT_ORDER;
    }

    try
    {
      this->mServiceAddMoveOrder = this->mNHPriv.advertiseService(
          lServiceNameAddMoveOrder.value(),
          &mars::agent::logical::common::behavior::SetCurrentOrder::
              serviceCallbackAddMoveOrder,
          this);
      this->mServiceAddTransportOrder = this->mNHPriv.advertiseService(
          lServiceNameAddTransportOrder.value(),
          &mars::agent::logical::common::behavior::SetCurrentOrder::
              serviceCallbackAddTransportOrder,
          this);
    }
    catch (const ros::InvalidNameException& e)
    {
      MARS_LOG_ERROR(e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  if (this->mOrderQueue.empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex>{this->mOrderQueueMutex};

  lResult =
      this->setOutput<std::shared_ptr<mars::agent::logical::common::Order>>(
          BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER, this->mOrderQueue.front());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<mars::common::Id>(
      BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER_ID,
      this->mOrderQueue.front()->getId());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  this->mOrderQueue.pop();

  lResult = this->setOutput<bool>(
      BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER_FLAG, true);
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

bool mars::agent::logical::common::behavior::SetCurrentOrder::
    serviceCallbackAddMoveOrder(
        mars_agent_logical_srvs::AddMoveOrder::Request& pRequest,
        mars_agent_logical_srvs::AddMoveOrder::Response& pResponse)
{
  mars_agent_logical_msgs::OrderStatus orderMsg;
  std::lock_guard<std::mutex>{this->mOrderQueueMutex};

  this->mOrderQueue.push(
      std::make_shared<mars::agent::logical::common::MoveOrder>(
          pRequest.move_order));

  orderMsg.order_id = pRequest.move_order.move_order_id;
  orderMsg.order_state =  mars_agent_logical_msgs::OrderStatus::ORDER_STATE_UNKNOWN;
  orderMsg.order_status =
      mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_WAITING;
  orderMsg.order_type =
      mars_agent_logical_msgs::OrderStatus::ORDER_TYPE_MOVE_ORDER;

  this->mOrderStatusPub.publish(orderMsg);

  return true;
}

bool mars::agent::logical::common::behavior::SetCurrentOrder::
    serviceCallbackAddTransportOrder(
        mars_agent_logical_srvs::AddTransportOrder::Request& pRequest,
        mars_agent_logical_srvs::AddTransportOrder::Response& pResponse)
{
  mars_agent_logical_msgs::OrderStatus orderMsg;
  std::lock_guard<std::mutex>{this->mOrderQueueMutex};

  this->mOrderQueue.push(
      std::make_shared<mars::agent::logical::common::TransportOrder>(
          pRequest.transport_order));

  orderMsg.order_id = pRequest.transport_order.transport_order_id;
  orderMsg.order_state = mars_agent_logical_msgs::OrderStatus::ORDER_STATE_UNKNOWN;
  orderMsg.order_status =
      mars_agent_logical_msgs::OrderStatus::ORDER_STATUS_WAITING;
  orderMsg.order_type =
      mars_agent_logical_msgs::OrderStatus::ORDER_TYPE_TRANSPORT_ORDER;

  this->mOrderStatusPub.publish(orderMsg);

  return true;
}
