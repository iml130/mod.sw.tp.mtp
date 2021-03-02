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

#include "mars_agent_logical_common/behavior/PublishOrderStatus.h"

static const std::string BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_STATUS = "order_status";
static const std::string
    BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_DESCRIPTION_ORDER_STATUS = "";

static const std::string BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_TYPE = "order_type";
static const std::string
    BEHAVIOR_PUBLISH_ORDER_STATUS_PARAMR_DESCRIPTION_ORDER_TYPE = "";

static const std::string BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_STATE = "order_state";
static const std::string
    BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_DESCRIPTION_ORDER_STATE = "";

static const std::string BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_ID = "order_id";
static const std::string
    BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_DESCRIPTION_ORDER_ID = "";

mars::agent::logical::common::behavior::PublishOrderStatus::PublishOrderStatus(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
  ros::NodeHandle nh;

  this->mOrderStatusPub =
      nh.advertise<mars_agent_logical_msgs::OrderStatus>("/order_status", 10, true);
}

BT::PortsList
mars::agent::logical::common::behavior::PublishOrderStatus::providedPorts()
{
  return {
      // current order status
      BT::InputPort<unsigned int>(
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_STATUS,
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_DESCRIPTION_ORDER_STATUS),
      // current order type
      BT::InputPort<unsigned int>(
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_TYPE,
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAMR_DESCRIPTION_ORDER_TYPE),
      // current order state
      BT::InputPort<unsigned int>(
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_STATE,
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_DESCRIPTION_ORDER_STATE),
      // current order id
      BT::InputPort<mars::common::Id>(
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_ID,
          BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_DESCRIPTION_ORDER_ID),
  };
}

BT::NodeStatus
mars::agent::logical::common::behavior::PublishOrderStatus::tick()
{
  mars_agent_logical_msgs::OrderStatus orderMsg;

  if (!getParams())
  {
    // could not read every param
    return BT::NodeStatus::FAILURE;
  }

  orderMsg.order_id = this->mCurrentOrderId.toMsg();
  orderMsg.order_state = this->mCurrentOrderState;
  orderMsg.order_status = this->mCurrentOrderStatus;
  orderMsg.order_type = this->mCurrentOrderType;

  this->mOrderStatusPub.publish(orderMsg);

  return BT::NodeStatus::SUCCESS;
}

bool mars::agent::logical::common::behavior::PublishOrderStatus::getParams()
{

  BT::Optional<unsigned int> getOrderStatusResult;
  BT::Optional<unsigned int> getOrderStateResult;
  BT::Optional<unsigned int> getOrderStateType;
  BT::Optional<mars::common::Id> getOrderId;

  // gets set to false if any param could not be found
  bool successful = true;

  // get order status
  getOrderStatusResult =
      getInput<unsigned int>(BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_STATUS);
  if (!getOrderStatusResult)
  {
    MARS_LOG_ERROR(getOrderStatusResult.error());
    successful = false;
  }
  else
  {
    this->mCurrentOrderStatus = getOrderStatusResult.value();
  }

  // get order status
  getOrderStateResult =
      getInput<unsigned int>(BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_STATE);
  if (!getOrderStateResult)
  {
    MARS_LOG_ERROR(getOrderStateResult.error());
    successful = false;
  }
  else
  {
    this->mCurrentOrderState = getOrderStateResult.value();
  }

  // get order type
  getOrderStateType =
      getInput<unsigned int>(BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_TYPE);
  if (!getOrderStateType)
  {
    MARS_LOG_ERROR(getOrderStateType.error());
    successful = false;
  }
  else
  {
    this->mCurrentOrderType = getOrderStateType.value();
  }

  // get order id
  getOrderId =
      getInput<mars::common::Id>(BEHAVIOR_PUBLISH_ORDER_STATUS_PARAM_ORDER_ID);
  if (!getOrderId)
  {
    MARS_LOG_ERROR(getOrderId.error());
    successful = false;
  }
  else
  {
    this->mCurrentOrderId = getOrderId.value();
  }
  return successful;
}
