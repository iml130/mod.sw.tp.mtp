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

#include "mars_agent_logical_common/behavior/PlanRoute.h"
#include "mars_common/Logger.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_core/Route.h"
#include "mars_routing_srvs/GetRoute.h"

static const std::string SERVICE_NAME_GET_ROUTE = "/GetRoute";

BT::NodeStatus mars::agent::logical::common::behavior::PlanRoute::tick()
{
  BT::Optional<mars::common::Id> lAgentId;
  BT::Optional<std::string> lRouter;
  BT::Optional<std::shared_ptr<mars::routing::common::topology::Entity>> lOrigin;
  BT::Optional<std::shared_ptr<mars::routing::common::topology::Entity>> lDestination;
  BT::Optional<ros::Duration> lDestinationReservationDuration;
  BT::Optional<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>
      lRobotProperties;
  BT::Optional<Eigen::Affine3d> lRobotPose;
  BT::Optional<ros::Time> lStartTime;
  BT::Result lResult;
  bool serviceCallSuccesful = false;

  mars::routing::core::IterationRoute* lRoutePtr;

  lAgentId = this->getInput<mars::common::Id>(BEHAVIOR_PLANROUTE_PARAM_NAME_AGENT_ID);
  if (!lAgentId)
  {
    MARS_LOG_ERROR(lAgentId.error());
    return BT::NodeStatus::FAILURE;
  }

  lRouter = this->getInput<std::string>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTER);
  if (!lRouter)
  {
    MARS_LOG_ERROR(lRouter.error());
    return BT::NodeStatus::FAILURE;
  }

  lOrigin = this->getInput<std::shared_ptr<mars::routing::common::topology::Entity>>(
      BEHAVIOR_PLANROUTE_PARAM_NAME_ORIGIN);
  if (!lOrigin)
  {
    MARS_LOG_ERROR(lOrigin.error());
    return BT::NodeStatus::FAILURE;
  }

  lDestination = this->getInput<std::shared_ptr<mars::routing::common::topology::Entity>>(
      BEHAVIOR_PLANROUTE_PARAM_NAME_DESTINATION);
  if (!lDestination)
  {
    MARS_LOG_ERROR(lDestination.error());
    return BT::NodeStatus::FAILURE;
  }

  lDestinationReservationDuration =
      this->getInput<ros::Duration>(BEHAVIOR_PLANROUTE_PARAM_NAME_DESTINATION_RESERVATION_DURATION);
  if (!lDestinationReservationDuration)
  {
    MARS_LOG_ERROR(lDestinationReservationDuration.error());
    return BT::NodeStatus::FAILURE;
  }

  lRobotProperties =
      this->getInput<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
          BEHAVIOR_PLANROUTE_PARAM_NAME_ROBOT_PROPERTIES);
  if (!lRobotProperties)
  {
    MARS_LOG_ERROR(lRobotProperties.error());
    return BT::NodeStatus::FAILURE;
  }

  lRobotPose = this->getInput<Eigen::Affine3d>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROBOT_POSE);
  if (!lRobotPose)
  {
    MARS_LOG_ERROR(lRobotPose.error());
    return BT::NodeStatus::FAILURE;
  }

  lStartTime = this->getInput<ros::Time>(BEHAVIOR_PLANROUTE_PARAM_NAME_TIME);
  if (!lStartTime)
  {
    MARS_LOG_ERROR(lStartTime.error());
    return BT::NodeStatus::FAILURE;
  }

  mars_routing_srvs::GetRoute lGetRouteService;
  lGetRouteService.request.route_request_type = lGetRouteService.request.GET_ROUTE;
  lGetRouteService.request.start_time.data =
      ros::Time::now() + ros::Duration(1); // lStartTime.value();
  lGetRouteService.request.origin.id = lOrigin.value()->getId().toMsg();
  lGetRouteService.request.origin.entity_type.entity_type = lOrigin.value()->getType();
  lGetRouteService.request.destination.id = lDestination.value()->getId().toMsg();
  lGetRouteService.request.destination.entity_type.entity_type = lDestination.value()->getType();

  lGetRouteService.request.destination_reservation_duration =
      lDestinationReservationDuration.value();

  lGetRouteService.request.robot_agent_properties = lRobotProperties.value()->toMsg();
  lGetRouteService.request.robot_agent_properties.robot_id = lAgentId.value().toMsg();
  lGetRouteService.request.robot_agent_orientation =
      lRobotPose.value().rotation().eulerAngles(2, 1, 0)[0];

  std::stringstream lStringStream;
  lStringStream << lRouter.value() << SERVICE_NAME_GET_ROUTE;
  std::string lServicePath = lStringStream.str();

  this->mGetRouteClient =
      mNodeHandle.serviceClient<mars_routing_srvs::GetRoute>(lServicePath, true);

  serviceCallSuccesful = this->mGetRouteClient.call(lGetRouteService);

  if (!serviceCallSuccesful)
  {
    // something went wrong while calling the service of the router
    MARS_LOG_ERROR("Service: " << SERVICE_NAME_GET_ROUTE << " was not reachable.")
    return BT::NodeStatus::FAILURE;
  }
  else if (lGetRouteService.response.result.result == mars_common_msgs::Result::RESULT_ERROR)
  {
    MARS_LOG_WARN("Service: " << SERVICE_NAME_GET_ROUTE << " reported an error ("
                              << lGetRouteService.response.result.description
                              << "). Propably the destination is currently not available. Retry.");
    return BT::NodeStatus::FAILURE;
  }

  if (lGetRouteService.request.origin.entity_type.entity_type >=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      lGetRouteService.request.origin.entity_type.entity_type <=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX &&
      lGetRouteService.request.destination.entity_type.entity_type >=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      lGetRouteService.request.destination.entity_type.entity_type <=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
  {
    lRoutePtr = new mars::routing::core::Route<mars::routing::common::topology::Vertex,
                                               mars::routing::common::topology::Edge,
                                               mars::routing::common::topology::Vertex>(
        mars::routing::core::Route<mars::routing::common::topology::Vertex,
                                   mars::routing::common::topology::Edge,
                                   mars::routing::common::topology::Vertex>(lGetRouteService));
  }

  if (!lRoutePtr->isValid())
  {
    return BT::NodeStatus::FAILURE;
  }

  mars_routing_msgs::Route msg;

  msg.path_id = lGetRouteService.response.path_id;
  msg.agent_id = lAgentId.value().toMsg();
  msg.robot_agent_properties = lRobotProperties.value()->toMsg();
  msg.route = lGetRouteService.response.route;

  this->mRoutePublisher.publish(msg);

  lResult =
      this->setOutput<mars::common::Id>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_ID, lRoutePtr->getId());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<mars::routing::core::IterationStep*>(
      BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_ALLOCATE_STEP, lRoutePtr->begin().getStep());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<mars::routing::core::IterationStep*>(
      BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_DEALLOCATE_STEP, lRoutePtr->begin().getStep());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<int>(BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE_STEP_COUNT,
                                 lRoutePtr->getStepCount());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<ros::Time>(BEHAVIOR_PLANROUTE_PARAM_NAME_TIME,
                                       lRoutePtr->getTravelInterval().getEndTime());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  //  lResult =
  //  this->setOutput<std::shared_ptr<mars::routing::core::IterationRoute>>(
  //      BEHAVIOR_PLANROUTE_PARAM_NAME_ROUTE, lRoutePtr);
  //  if (!lResult)
  //  {
  //    MARS_LOG_ERROR(lResult.error());
  //    return BT::NodeStatus::FAILURE;
  //  }

  return BT::NodeStatus::SUCCESS;
}

void mars::agent::logical::common::behavior::PlanRoute::halt()
{
  if (this->mGetRouteClient)
  {
    this->mGetRouteClient.shutdown();
    // FIXME: At some point reservations need to be deleted, once Router finds a
    // successful Route!
  }
}
