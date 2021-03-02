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

#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_PLANROUTE_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_PLANROUTE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <ros/ros.h>
#include <ros/service_client.h>

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_core/IterationRoute.h"
#include "mars_routing_core/Step.h"

#include <mars_routing_msgs/Route.h>

static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_AGENT_ID = "agent_id";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTER = "router";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ORIGIN = "origin";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_DESTINATION = "destination";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_DESTINATION_RESERVATION_DURATION =
    "destination_reservation_duration";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROBOT_PROPERTIES = "robot_properties";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROBOT_POSE = "robot_pose";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_TIME = "time";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_ID = "route_id";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_ALLOCATE_STEP = "route_allocate_step";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_DEALLOCATE_STEP =
    "route_deallocate_step";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_STEP_COUNT = "route_step_count";
static const std::string BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE = "route";

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
class PlanRoute : public BT::AsyncActionNode
{
public:
  PlanRoute(const std::string& pName, const BT::NodeConfiguration& pConfig)
      : BT::AsyncActionNode(pName, pConfig)
  {
    ros::NodeHandle nh;
    this->mRoutePublisher = nh.advertise<mars_routing_msgs::Route>("/route", 1, true);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<mars::common::Id>(BEHAVIOR_PLANROUTE_PARAM_NAME_AGENT_ID),
            BT::InputPort<std::string>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTER),
            BT::InputPort<std::shared_ptr<mars::routing::common::topology::Entity>>(
                BEHAVIOR_PLANROUTE_PARAM_NAME_ORIGIN),
            BT::InputPort<std::shared_ptr<mars::routing::common::topology::Entity>>(
                BEHAVIOR_PLANROUTE_PARAM_NAME_DESTINATION),
            BT::InputPort<ros::Duration>(
                BEHAVIOR_PLANROUTE_PARAM_NAME_DESTINATION_RESERVATION_DURATION),
            BT::InputPort<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
                BEHAVIOR_PLANROUTE_PARAM_NAME_ROBOT_PROPERTIES),
            BT::InputPort<Eigen::Affine3d>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROBOT_POSE),
            BT::OutputPort<mars::common::Id>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_ID),
            BT::OutputPort<mars::routing::core::IterationStep*>(
                BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_ALLOCATE_STEP),
            BT::OutputPort<mars::routing::core::IterationStep*>(
                BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_DEALLOCATE_STEP),
            BT::OutputPort<int>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_STEP_COUNT),
            BT::OutputPort<std::shared_ptr<mars::routing::core::IterationRoute>>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE),
            BT::BidirectionalPort<ros::Time>(BEHAVIOR_PLANROUTE_PARAM_NAME_TIME)};
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  ros::ServiceClient mGetRouteClient;
  ros::NodeHandle mNodeHandle;
  ros::Publisher mRoutePublisher;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_PLANROUTE_H
