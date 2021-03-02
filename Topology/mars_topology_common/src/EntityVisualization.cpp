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

#include "mars_topology_common/utility/EntityVisualization.h"

static const std::string INTERACTIVE_MARKER_MENU_NAME_INFOS = "NodeInfos";
static const std::string INTERACTIVE_MARKER_MENU__NAME_CONTAINER = "ContainerInfos";
static const std::string INTERACTIVE_MARKER_MENU_NAME_Infos_Container_ID = "Container ID";
static const std::string INTERACTIVE_MARKER_MENU_NAME_Infos_Container_NAME = "Container name";
static const std::string INTERACTIVE_MARKER_MENU_Node = "Node";

static const std::string CONTAINER_NAME_TEXT_INVALID = "Container name not set!";
static const std::string CONTAINER_ID_TEXT_INVALID = "Container Id not set!";
static const std::string ENTITY_NAME_TEXT_INVALID = "Name not set!";
static const std::string ENTITY_ID_TEXT_INVALID = "ID not set!";

mars::topology::common::utility::EntityVisualization::EntityVisualization()
    : mEngine(std::random_device{}())
{
}

visualization_msgs::Marker
mars::topology::common::utility::EntityVisualization::createMarkerLineStrip(
    const geometry_msgs::PolygonStamped& footprint, const std::string& markerFrame,
    const std::string& entityNamespace)
{
  visualization_msgs::Marker entityMarker;
  entityMarker.header.frame_id = markerFrame;
  entityMarker.header.stamp = ros::Time::now();
  entityMarker.ns = entityNamespace;

  // initialize Quaternion in a normalized form
  entityMarker.pose.orientation.x = 0;
  entityMarker.pose.orientation.y = 0;
  entityMarker.pose.orientation.z = 0;
  entityMarker.pose.orientation.w = 1;

  entityMarker.id = this->mDistribution(this->mEngine);

  entityMarker.type = visualization_msgs::Marker::LINE_STRIP;
  entityMarker.action = visualization_msgs::Marker::ADD;
  for (auto& i : footprint.polygon.points)
  {
    // Convert Point32 to Point
    geometry_msgs::Point p;
    p.x = i.x;
    p.y = i.y;
    p.z = i.z;
    entityMarker.points.push_back(p);
  }
  auto i = footprint.polygon.points.begin();
  geometry_msgs::Point p;
  p.x = i->x;
  p.y = i->y;
  p.z = i->z;
  entityMarker.points.push_back(p);
  entityMarker.scale.x = .1;
  entityMarker.color.r = 0;
  entityMarker.color.g = 0;
  entityMarker.color.b = 1.0f;
  entityMarker.color.a = 1.0;
  entityMarker.lifetime = ros::Duration(0);

  return entityMarker;
}

void mars::topology::common::utility::EntityVisualization::createAndAddMarkerArrow(
    const geometry_msgs::PointStamped& edgeCoordinate, const std::string& frame,
    const mars::common::Id& entityId, const std::string& nodeNamespace,
    const mars::common::Id& origin, const mars::common::Id& originContainerId,
    const geometry_msgs::PointStamped& originCoordinate, const mars::common::Id& destination,
    const mars::common::Id& targetContainerId, const geometry_msgs::PointStamped& targetCoordinate,
    mars::topology::common::DirectionType direction, const std::pair<bool, bool> serviceCalls,
    visualization_msgs::MarkerArray& pMarkerArray)
{
  geometry_msgs::PointStamped lTargetCoordinate, lOriginCoordinate;

  lTargetCoordinate = this->createCoordinatePoint(serviceCalls.first, targetCoordinate, destination,
                                                  targetContainerId);
  lOriginCoordinate =
      this->createCoordinatePoint(serviceCalls.second, originCoordinate, origin, originContainerId);

  // draw an arrow of the edge deppending of its direction type
  switch (direction)
  {
  case (mars::topology::common::DirectionType::unidirectional):
  {
    pMarkerArray.markers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, lOriginCoordinate, lTargetCoordinate,
                                      mars::topology::common::utility::ArrowColor::redColor));
    break;
  }
  case (mars::topology::common::DirectionType::unidirectional_reversed):
  {
    pMarkerArray.markers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, lTargetCoordinate, lOriginCoordinate,
                                      mars::topology::common::utility::ArrowColor::redColor));
    break;
  }
  case (mars::topology::common::DirectionType::bidirectional):
  {
    pMarkerArray.markers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, edgeCoordinate, lTargetCoordinate,
                                      mars::topology::common::utility::ArrowColor::greenColor));
    pMarkerArray.markers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, edgeCoordinate, lOriginCoordinate,
                                      mars::topology::common::utility::ArrowColor::greenColor));
    break;
  }
  default:
  {
    break;
  }
  }
}

std::vector<visualization_msgs::Marker>
mars::topology::common::utility::EntityVisualization::createAndAddMarkerArrow(
    const geometry_msgs::PointStamped& edgeCoordinate, const std::string& frame,
    const mars::common::Id& entityId, const std::string& nodeNamespace,
    const mars::common::Id& origin, const mars::common::Id& originContainerId,
    const geometry_msgs::PointStamped& originCoordinate, const mars::common::Id& destination,
    const mars::common::Id& targetContainerId, const geometry_msgs::PointStamped& targetCoordinate,
    mars::topology::common::DirectionType direction, const std::pair<bool, bool> serviceCalls)
{
  geometry_msgs::PointStamped lTargetCoordinate, lOriginCoordinate;
  std::vector<visualization_msgs::Marker> lDirectionMarkers;

  lTargetCoordinate = this->createCoordinatePoint(serviceCalls.first, targetCoordinate, destination,
                                                  targetContainerId);
  lOriginCoordinate =
      this->createCoordinatePoint(serviceCalls.second, originCoordinate, origin, originContainerId);

  // draw an arrow of the edge deppending of its direction type
  switch (direction)
  {
  case (mars::topology::common::DirectionType::unidirectional):
  {
    lDirectionMarkers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, lOriginCoordinate, lTargetCoordinate,
                                      mars::topology::common::utility::ArrowColor::redColor));
    break;
  }
  case (mars::topology::common::DirectionType::unidirectional_reversed):
  {
    lDirectionMarkers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, lTargetCoordinate, lOriginCoordinate,
                                      mars::topology::common::utility::ArrowColor::redColor));
    break;
  }
  case (mars::topology::common::DirectionType::bidirectional):
  {
    lDirectionMarkers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, edgeCoordinate, lTargetCoordinate,
                                      mars::topology::common::utility::ArrowColor::greenColor));
    lDirectionMarkers.push_back(
        this->createSingleMarkerArrow(frame, nodeNamespace, edgeCoordinate, lOriginCoordinate,
                                      mars::topology::common::utility::ArrowColor::greenColor));
    break;
  }
  default:
  {
    break;
  }
  }

  return lDirectionMarkers;
}

geometry_msgs::PointStamped
mars::topology::common::utility::EntityVisualization::createCoordinatePoint(
    const bool& pServiceCall, const geometry_msgs::PointStamped& pCoordinatePoint,
    const mars::common::Id& pEntityId, const mars::common::Id& pContainerId)
{
  // check if a service is needed to be called
  if (!pServiceCall)
  {
    return pCoordinatePoint;
  }
  else
  {
    geometry_msgs::PointStamped lCoordinatePoint;
    mars_topology_srvs::GetCoordinate lServiceCoordinate;
    lServiceCoordinate.request.entity_id = pEntityId.toMsg();
    std::string lServerId;
    lServerId = pContainerId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC);
    std::string destinationServiceName = "/topology/container_" + lServerId + "/get_coordinate";

    if (ros::service::exists(destinationServiceName, true))
    {
      ros::service::call(destinationServiceName, lServiceCoordinate);

      lCoordinatePoint.header.frame_id = lServiceCoordinate.response.point.header.frame_id;
      lCoordinatePoint.point.x = lServiceCoordinate.response.point.point.x;
      lCoordinatePoint.point.y = lServiceCoordinate.response.point.point.y;
      lCoordinatePoint.point.z = lServiceCoordinate.response.point.point.z;

      return lCoordinatePoint;
    }
  }
}

visualization_msgs::Marker
mars::topology::common::utility::EntityVisualization::createSingleMarkerArrow(
    const std::string& pMarkerFrame, const std::string& pEntityNamespace,
    const geometry_msgs::PointStamped& pTailCoordinate,
    const geometry_msgs::PointStamped& pHeadCoordinate,
    const mars::topology::common::utility::ArrowColor pColor)
{
  visualization_msgs::Marker entityMarker;

  // fill marker msg
  entityMarker.header.frame_id = pMarkerFrame;
  entityMarker.header.stamp = ros::Time::now();
  entityMarker.ns = pEntityNamespace;
  entityMarker.id = this->mDistribution(this->mEngine);
  entityMarker.type = visualization_msgs::Marker::ARROW;
  entityMarker.action = visualization_msgs::Marker::ADD;

  double deltaX, deltaY, deltaZ, deltaAlpha, distance;

  deltaX = pHeadCoordinate.point.x - pTailCoordinate.point.x;
  deltaY = pHeadCoordinate.point.y - pTailCoordinate.point.y;
  deltaZ = pHeadCoordinate.point.z - pTailCoordinate.point.z;
  deltaAlpha = std::atan2(deltaY, deltaX);

  distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

  tf2::Quaternion quaternion;

  switch (pColor)
  {
  case mars::topology::common::utility::ArrowColor::redColor:
  {
    // set start point of the arrow to origin of the entity tf
    entityMarker.pose.position.x = pTailCoordinate.point.x + .25 * deltaX / distance;
    entityMarker.pose.position.y = pTailCoordinate.point.y + .25 * deltaY / distance;
    entityMarker.pose.position.z = 0;

    entityMarker.color.r = 1.0;
    entityMarker.color.g = 0;
    entityMarker.color.b = 0;
    break;
  }
  case mars::topology::common::utility::ArrowColor::greenColor:
  {
    // set start point of the arrow to origin of the entity tf
    entityMarker.pose.position.x = pTailCoordinate.point.x;
    entityMarker.pose.position.y = pTailCoordinate.point.y;
    entityMarker.pose.position.z = 0;

    entityMarker.color.r = 0;
    entityMarker.color.g = 1.0;
    entityMarker.color.b = 0;
    break;
  }
  default:
  {
    break;
  }
  }

  quaternion.setEuler(0, 0, deltaAlpha);

  entityMarker.pose.orientation = tf2::toMsg(quaternion);

  entityMarker.scale.x = sqrt((deltaX * deltaX + deltaY * deltaY)) - .40;

  entityMarker.scale.y = 0.2;
  entityMarker.scale.z = 0.01;

  entityMarker.color.a = 1.0;

  entityMarker.lifetime = ros::Duration(0);

  return entityMarker;
}

visualization_msgs::Marker mars::topology::common::utility::EntityVisualization::createMarkerPoint(
    const geometry_msgs::PointStamped& pPosition, const std::string& markerFrame,
    const std::string& entityNamespace)
{
  visualization_msgs::Marker entityMarker;
  entityMarker.header.frame_id = markerFrame;
  entityMarker.header.stamp = ros::Time::now();
  entityMarker.ns = entityNamespace;

  // initialize Quaternion in a normalized form
  entityMarker.pose.position.x = pPosition.point.x;
  entityMarker.pose.position.y = pPosition.point.y;
  entityMarker.pose.position.z = pPosition.point.z;
  entityMarker.pose.orientation.x = 0;
  entityMarker.pose.orientation.y = 0;
  entityMarker.pose.orientation.z = 0;
  entityMarker.pose.orientation.w = 1;

  entityMarker.id = this->mDistribution(this->mEngine);

  entityMarker.type = visualization_msgs::Marker::CYLINDER;
  entityMarker.action = visualization_msgs::Marker::ADD;
  entityMarker.scale.x = .2;
  entityMarker.scale.y = .2;
  entityMarker.scale.z = .05;
  entityMarker.color.r = 0;
  entityMarker.color.g = 0;
  entityMarker.color.b = 1.0f;
  entityMarker.color.a = 1.0;
  entityMarker.lifetime = ros::Duration(0);

  return entityMarker;
}
