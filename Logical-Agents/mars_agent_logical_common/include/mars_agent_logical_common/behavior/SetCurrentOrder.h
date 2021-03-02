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

#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTORDER_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTORDER_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <mutex>
#include <queue>
#include <ros/ros.h>

#include <boost/optional.hpp>

#include <mars_agent_logical_srvs/AddMoveOrder.h>
#include <mars_agent_logical_srvs/AddServiceOrder.h>
#include <mars_agent_logical_srvs/AddTransportOrder.h>

#include <mars_agent_logical_msgs/OrderStatus.h>

#include <mars_agent_logical_common/TransportOrder.h>

#include <mars_agent_logical_common/Order.h>

#include <mars_common/Id.h>


static const std::string BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER = "order";
static const std::string BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER_ID = "order_id";
static const std::string BEHAVIOR_SETCURRENTORDER_PARAM_NAME_ORDER_FLAG = "new_order_flag";

static const std::string BEHAVIOR_SETCURRENTORDER_PARAM_NAME_SERVICE_NAME_ADD_MOVE_ORDER =
    "service_name_add_move_order";
static const std::string BEHAVIOR_SETCURRENTORDER_PARAM_NAME_SERVICE_NAME_ADD_TRANSPORT_ORDER =
    "service_name_add_transport_order";

static const std::string BEHAVIOR_SETCURRENTORDER_DEFAULT_SERVICE_NAME_ADD_MOVE_ORDER =
    "add_move_order";
static const std::string BEHAVIOR_SETCURRENTORDER_DEFAULT_SERVICE_NAME_ADD_TRANSPORT_ORDER =
    "add_transport_order";

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
namespace behavior
{
class SetCurrentOrder : public BT::SyncActionNode
{
public:
  SetCurrentOrder(const std::string& pName, const BT::NodeConfiguration& pConfig);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private: // functions
  bool serviceCallbackAddMoveOrder(mars_agent_logical_srvs::AddMoveOrder::Request& pRequest,
                                   mars_agent_logical_srvs::AddMoveOrder::Response& pResponse);

  bool
  serviceCallbackAddTransportOrder(mars_agent_logical_srvs::AddTransportOrder::Request& pRequest,
                                   mars_agent_logical_srvs::AddTransportOrder::Response& pResponse);

private: // variables
  ros::NodeHandle mNHPriv;

  ros::Publisher mOrderStatusPub;

  boost::optional<ros::ServiceServer> mServiceAddMoveOrder;
  boost::optional<ros::ServiceServer> mServiceAddTransportOrder;

  std::queue<std::shared_ptr<mars::agent::logical::common::Order>> mOrderQueue;
  mutable std::mutex mOrderQueueMutex;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTTIME_H
