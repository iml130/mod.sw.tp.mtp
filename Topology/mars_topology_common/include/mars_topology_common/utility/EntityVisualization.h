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

#ifndef ENTITYVISUALIZATION_H
#define ENTITYVISUALIZATION_H

// std C++ includes
#include <math.h>
#include <random>
// ros includes
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

// ros msgs includes
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// own includes
#include <mars_common/Id.h>
#include <mars_topology_common/TopologyEntityType.h>

#include <mars_topology_srvs/GetCoordinate.h>

namespace mars
{
namespace topology
{
namespace common
{
namespace utility
{

enum class ArrowColor : int
{
  redColor = 0,
  greenColor = 1,
  blueColor = 2
};

class EntityVisualization
{
public:
  EntityVisualization();

  void createAndAddMarkerArrow(const geometry_msgs::PointStamped& edgeCoordinate,
                               const std::string& frame, const mars::common::Id& entityId,
                               const std::string& nodeNamespace, const mars::common::Id& origin,
                               const mars::common::Id& originContainerId,
                               const geometry_msgs::PointStamped& originCoordinate,
                               const mars::common::Id& destination,
                               const mars::common::Id& targetContainerId,
                               const geometry_msgs::PointStamped& targetCoordinate,
                               mars::topology::common::DirectionType direction,
                               const std::pair<bool, bool> serviceCalls,
                               visualization_msgs::MarkerArray& pMarkerArrows);

  std::vector<visualization_msgs::Marker> createAndAddMarkerArrow(
      const geometry_msgs::PointStamped& edgeCoordinate, const std::string& frame,
      const mars::common::Id& entityId, const std::string& nodeNamespace,
      const mars::common::Id& origin, const mars::common::Id& originContainerId,
      const geometry_msgs::PointStamped& originCoordinate, const mars::common::Id& destination,
      const mars::common::Id& targetContainerId,
      const geometry_msgs::PointStamped& targetCoordinate,
      mars::topology::common::DirectionType direction, const std::pair<bool, bool> serviceCalls);

  visualization_msgs::Marker createMarkerLineStrip(const geometry_msgs::PolygonStamped& footprint,
                                                   const std::string& markerFrame,
                                                   const std::string& entityNamespace);

  visualization_msgs::Marker createMarkerPoint(const geometry_msgs::PointStamped& pPosition,
                                               const std::string& markerFrame,
                                               const std::string& entityNamespace);

  /**
   * @brief Create a single Marker Arrow object
   *
   * @param pMarkerFrame the frame needed
   * @param pEntityNamespace
   * @param pTailCoordinate the point from which the arrow begins
   * @param pHeadCoordinate the point where the arrow ends
   * @param pColor the color of the arrow
   * @return visualization_msgs::Marker
   */
  visualization_msgs::Marker
  createSingleMarkerArrow(const std::string& pMarkerFrame, const std::string& pEntityNamespace,
                          const geometry_msgs::PointStamped& pTailCoordinate,
                          const geometry_msgs::PointStamped& pHeadCoordinate,
                          const mars::topology::common::utility::ArrowColor pColor);
  /**
   * @brief Create a Coordinate Point object either from a service call od not
   *
   * @param pServiceCall is true if a service call is needed in order to dertemine the wanted Point
   * @param pCoordinatePoint is the wanted point if the @param pServiceCal is set false
   * @param pEntityId is the id of the wanted Point
   * @param pContainerId is the container id from which the service needed to be called
   * @return geometry_msgs::PointStamped
   */
  geometry_msgs::PointStamped
  createCoordinatePoint(const bool& pServiceCall,
                        const geometry_msgs::PointStamped& pCoordinatePoint,
                        const mars::common::Id& pEntityId, const mars::common::Id& pContainerId);

private:
  /**
   * @brief mEngine Engine for random numbers in order to generate non
   * overlapping ids for the visualization marker.
   */
  std::mt19937 mEngine;
  /**
   * @brief mDistribution Distribution type to be generated by the random
   * engine.
   */
  std::uniform_int_distribution<uint32_t> mDistribution;
};
} // namespace utility
} // namespace common
} // namespace topology
} // namespace mars
#endif // ENTITYVISUALIZATION_H
