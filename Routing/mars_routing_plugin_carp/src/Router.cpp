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

#include "mars_common/Id.h"

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_common/utility/AffineProfile.h"

#include "mars_routing_plugin_carp/Route.h"
#include "mars_routing_plugin_carp/Router.h"

#include "mars_common/Logger.h"

PLUGINLIB_EXPORT_CLASS(mars::routing::plugin::carp::Router, mars::routing::core::Router)

mars::routing::plugin::carp::Router::Router() {}

void mars::routing::plugin::carp::Router::initialize(ros::console::levels::Level pLogLevel)
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, pLogLevel);

  MARS_LOG_DEBUG("Router initialized.");
}

bool mars::routing::plugin::carp::Router::getRoute(mars_routing_srvs::GetRoute::Request& req,
                                                   mars_routing_srvs::GetRoute::Response& res)
{
  bool lTestRoute = req.route_request_type == req.GET_TEST_ROUTE;
  double lRobotAgentOrientation = req.robot_agent_orientation;
  mars::agent::physical::common::RobotAgentProperties lRobotAgentProperties(
      req.robot_agent_properties);

  mars::common::Id lOriginId(req.origin.id);
  mars::common::Id lDestinationId(req.destination.id);

  ros::Time lStartTime(req.start_time.data);
  ros::Duration lDestinationReservationDuration(req.destination_reservation_duration);

  MARS_LOG_DEBUG("Requested Routing with start time " << lStartTime);

  mars::common::Id lAgentId(req.robot_agent_properties.robot_id);

  if (req.origin.entity_type.entity_type >=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      req.origin.entity_type.entity_type <=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX &&
      req.destination.entity_type.entity_type >=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      req.destination.entity_type.entity_type <=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
  {
    mars::routing::common::topology::Vertex lOrigin(lOriginId);
    mars::routing::common::topology::Vertex lDestination(lDestinationId);
    mars::routing::plugin::carp::Route<
        mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
        mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>
        lRoute(lOrigin, lDestination, lRobotAgentProperties, lRobotAgentOrientation, lStartTime,
               lDestinationReservationDuration);

    lRoute.mergeIntoServiceResponse(res);

    if (!lTestRoute && lRoute.isValid() && !this->reserveRoute(lRoute, lAgentId))
    {
      res.result.result = res.result.RESULT_ERROR;
    }
  }
  else if (req.origin.entity_type.entity_type >=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
           req.origin.entity_type.entity_type <=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX &&
           req.destination.entity_type.entity_type >=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
           req.destination.entity_type.entity_type <=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX)
  {
    mars::routing::common::topology::Vertex lOrigin(lOriginId);
    mars::routing::common::topology::Edge lDestination(lDestinationId);
    mars::routing::plugin::carp::Route<
        mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
        mars::routing::common::topology::Edge, mars::routing::common::utility::AffineProfile>
        lRoute(lOrigin, lDestination, lRobotAgentProperties, lRobotAgentOrientation, lStartTime,
               lDestinationReservationDuration);

    lRoute.mergeIntoServiceResponse(res);

    if (!lTestRoute && lRoute.isValid() && !this->reserveRoute(lRoute, lAgentId))
    {
      res.result.result = res.result.RESULT_ERROR;
    }
  }
  else if (req.origin.entity_type.entity_type >=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
           req.origin.entity_type.entity_type <=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX &&
           req.destination.entity_type.entity_type >=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
           req.destination.entity_type.entity_type <=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
  {
    mars::routing::common::topology::Edge lOrigin(lOriginId);
    mars::routing::common::topology::Vertex lDestination(lDestinationId);
    mars::routing::plugin::carp::Route<
        mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
        mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>
        lRoute(lOrigin, lDestination, lRobotAgentProperties, lRobotAgentOrientation, lStartTime,
               lDestinationReservationDuration);

    lRoute.mergeIntoServiceResponse(res);

    if (!lTestRoute && lRoute.isValid() && !this->reserveRoute(lRoute, lAgentId))
    {
      res.result.result = res.result.RESULT_ERROR;
    }
  }
  else if (req.origin.entity_type.entity_type >=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
           req.origin.entity_type.entity_type <=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX &&
           req.destination.entity_type.entity_type >=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
           req.destination.entity_type.entity_type <=
               mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX)
  {
    mars::routing::common::topology::Edge lOrigin(lOriginId);
    mars::routing::common::topology::Edge lDestination(lDestinationId);
    mars::routing::plugin::carp::Route<
        mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
        mars::routing::common::topology::Edge, mars::routing::common::utility::AffineProfile>
        lRoute(lOrigin, lDestination, lRobotAgentProperties, lRobotAgentOrientation, lStartTime,
               lDestinationReservationDuration);

    lRoute.mergeIntoServiceResponse(res);

    if (!lTestRoute && lRoute.isValid() && !this->reserveRoute(lRoute, lAgentId))
    {
      res.result.result = res.result.RESULT_ERROR;
    }

    if (lRoute.isValid())
    {
      MARS_LOG_DEBUG(
          "Get " << ((lTestRoute) ? "test route " : "route ") << "for agent "
                 << lAgentId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                 << " with path id "
                 << lRoute.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                 << " from origin "
                 << lOriginId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                 << " to destination "
                 << lDestinationId.getUUIDAsString(
                        mars::common::Id::UUIDFormat::HEXDEC_SPLIT));
    }
    else
    {
      MARS_LOG_DEBUG("Get " << ((lTestRoute) ? "test route " : "route ") << "failed for agent "
                            << lAgentId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                            << " from origin "
                            << lOriginId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                            << " to destination "
                            << lDestinationId.getUUIDAsString(
                                   mars::common::Id::UUIDFormat::HEXDEC_SPLIT));
    }
  }
  else
  {
    MARS_LOG_WARN("Invalid entity types");
  }

  return true;
}

bool mars::routing::plugin::carp::Router::reserveRoute(
    mars::routing::plugin::carp::PlanningRoute& pRoute, const mars::common::Id& pAgentId) const
{
  return pRoute.reserve(pAgentId);
}
