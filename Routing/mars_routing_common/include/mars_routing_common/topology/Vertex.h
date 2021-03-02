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

#ifndef MARS_ROUTING_COMMON_TOPOLOGY_VERTEX_H
#define MARS_ROUTING_COMMON_TOPOLOGY_VERTEX_H

#include "mars_common/Id.h"
#include "mars_common/TimeInterval.h"
#include <mars_common/Logger.h>

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_common/geometry/Footprint.h"
#include "mars_routing_common/Cache.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_common/topology/OccupationDurationKey.h"
#include "mars_routing_common/topology/ShortestOccupationDurationKey.h"
#include "mars_routing_common/topology/VertexInterface.h"

#include "mars_topology_common/TopologyEntityRestrictions.h"
#include "mars_topology_msgs/TopologyEntity.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/TimeInfo.h"

#include <eigen3/Eigen/Core>
#include <memory>

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
class Edge;

/**
 * @class Vertex
 */
class Vertex : public mars::routing::common::topology::Entity
{
public:
  /**
   * @brief Default Vertex Constructor with cache.
   * @param pId Unique Id of the vertex. Has to be unique in regards to other
   * topology entities, too.
   * @param pCache True, if the vertex attributes should be statically cached
   * and shared.
   */
  Vertex(const mars::common::Id& pId, const bool pCache = true);

  /**
   * @brief Complex getter for vertex type.
   */
  int getType();

  /**
   * @return Current Location of the vertex in reference to the map frame.
   */
  const boost::optional<Eigen::Vector3d> getLocation();

  /**
   * @brief Set the location of the vertex directly.
   * Only works, if it hasn't been set or requested already prior to this call.
   * @param pLocation Location of the vertex in reference to the map frame.
   */
  void setLocation(const Eigen::Vector3d& pLocation);

  /**
   * @brief Complex getter for the footprint of this vertex.
   * Requests the footprint from a connected topology node, if needed.
   * @return Footprint of the vertex in absolute coordinates (world frame).
   * @return boost::none, if service and value are unavailable.
   */
  const boost::optional<mars::common::geometry::Footprint&> getFootprint();

  /**
   * @brief Complex getter for the restrictions of this vertex.
   * Requests the restrictions from a connected topology node, if needed.
   * @return Restrictions for traversing this vertex.
   * @return boost::none, if service and value are unavailable.
   */
  const boost::optional<mars::topology::common::TopologyEntityRestrictions&> getRestrictions();

  /**
   * @brief Complex getter for the edges going into this vertex.
   * Requests the edges from a connected topology node, if needed.
   * @return Ingoing edges for this vertex in the topology.
   * @return boost::none, if service and value are unavailable.
   */
  boost::optional<std::vector<mars::routing::common::topology::Edge>&> getIngoingEdges();

  /**
   * @brief Complex getter for the edges going out of this vertex.
   * Requests the edges from a connected topology node, if needed.
   * @return Outgoing edges for this vertex in the topology.
   * @return boost::none, if service and value are unavailable.
   */
  boost::optional<std::vector<mars::routing::common::topology::Edge>&> getOutgoingEdges();

  /**
   * @brief Complex getter for the lock state of this vertex.
   * Evaluates responded list of locked timeframes from a service.
   * @return True, if this vertex is locked at the current time.
   * @return boost::none, if service and value are unavailable.
   */
  boost::optional<bool> isLocked();

  /**
   * @brief Checks, if this vertex can be traversed with certain robot
   * properties.
   * @param pRAP Properties of the robot.
   * @return True, if this vertex can be traveresed by a robot with the given
   * properties.
   */
  bool isTraversable(const mars::agent::physical::common::RobotAgentProperties& pRAP);

  /**
   * @brief Complex getter for the edges going out of this vertex, which are
   * traversable by certain robots.
   * @param pRAP Properties of the robot.
   * @return Outgoing edges for this vertex in the topology, which are
   * traversable by a robot with the given properties.
   */
  std::vector<mars::routing::common::topology::Edge>
  getTraversableOutgoingEdges(const mars::agent::physical::common::RobotAgentProperties& pRAP);

  /**
   * @brief Complex getter for the edges going into this vertex, which are
   * traversable by certain robots.
   * @param pRAP Properties of the robot.
   * @return Ingoing edges for this vertex in the topology, which are
   * traversable by a robot with the given properties.
   */
  std::vector<mars::routing::common::topology::Edge>
  getTraversableIngoingEdges(const mars::agent::physical::common::RobotAgentProperties& pRAP);

  /**
   * @brief Generalized function for getTraversableOutgoingEdges().
   * Redirects call to class specific function.
   * @param pRAP Properties of the robot.
   * @return Outgoing edges for this vertex in the topology, which are
   * traversable by a robot with the given properties.
   */
  std::vector<mars::routing::common::topology::Edge>
  getTraversableTargets(const mars::agent::physical::common::RobotAgentProperties& pRAP);

  /**
   * @brief Complex getter for the free time intervals of this vertex.
   * Requests the time intervals from a connected topology node, if needed.
   * @return Free time intervals of the vertex in the topology.
   * @return boost::none, if service is unavailable.
   */
  boost::optional<std::vector<mars::common::TimeInterval>>
  getFreeTimeIntervals(const ros::Time& pStartTime);

  /**
   * @brief Complex getter for the shortest occupation duration while traversing
   * the vertex from an edge to the closest target. Requests topology entity
   * locations and footprints from a connected topology node, if needed.
   * Calculates the occupation duration and stores it in a map for lookup.
   * @return Shortest possible occupation duration on the vertex for certain
   * robot parameters.
   * @return boost::none, if service and lookup value are unavailable.
   */
  boost::optional<ros::Duration> getShortestOccupationDuration(
      mars::routing::common::topology::Edge& pIngoingEdge,
      const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
      mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
          pTimeInfo);

  /**
   * @brief Complex getter for the real occupation duration while traversing the
   * vertex from one edge to another. Requests topology entity locations and
   * footprints from a connected topology node, if needed. Calculates the
   * occupation duration and stores it in a map for lookup.
   * @return Occupation duration on the vertex for certain robot parameters when
   * traversing it.
   * @return boost::none, if service and lookup value are unavailable.
   */
  boost::optional<ros::Duration> getOccupationDuration(
      mars::routing::common::topology::Edge& pIngoingEdge,
      mars::routing::common::topology::Edge& pOutgoingEdge,
      const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
      mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
          pTimeInfo);

  /**
   * @brief Adds a reservation to this vertex.
   * @param pAgentId Agent ID of the agent which wants to reservate the entity.
   * @param pPathId Path ID of the current agent path.
   * @param pTimeInterval Interval of the reservation.
   * @return true on success, false otherwise.
   */
  bool addReservation(const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
                      const mars::common::TimeInterval& pTimeInterval);

  /**
   * @brief Deletes a reservation from this vertex.
   * @param pAgentId Agent ID of the agent which causes the deletion.
   * @param pPathId Path ID of the agent path.
   * @return true on success, false otherwise.
   */
  bool deleteReservation(const mars::common::Id& pAgentId, const mars::common::Id& pPathId);

  /**
   * @brief TODO: documentation, implementation
   */
  bool allocate(const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
                const mars::common::TimeInterval& pAllocationInterval,
                const ros::Duration& pWaitForGoal = ros::Duration(0, 0),
                const actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>::
                    SimpleDoneCallback& pDoneCallback = actionlib::SimpleActionClient<
                        mars_topology_actions::AllocateEntityAction>::SimpleDoneCallback(),
                const actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>::
                    SimpleActiveCallback& pActiveCallback = actionlib::SimpleActionClient<
                        mars_topology_actions::AllocateEntityAction>::SimpleActiveCallback(),
                const actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>::
                    SimpleFeedbackCallback& pFeedbackCallback = actionlib::SimpleActionClient<
                        mars_topology_actions::AllocateEntityAction>::SimpleFeedbackCallback());

  /**
   * @brief Frees this vertex, in order to make it usable by other agents.
   * @param pAgentId Agent ID of the agent which shall "do" the deallocation.
   * @return true on success, false otherwise.
   */
  bool deallocate(const mars::common::Id& pAgentId, const mars::common::Id& pPathId);

  /**
   * @brief Checks if the passed footprint is fully inside this entitie's
   * footprint.
   * @param pFootprint Footprint to be checked.
   * @return true, if passed footprint is fully within the footprint of this
   * entity.
   */
  bool contains(const mars::common::geometry::Footprint& pFootprint);

  /**
   * @brief Checks if the passed footprint is fully inside this entitie's
   * footprint.
   * @param pLocation Location of the robot.
   * @param pRAP Properties of the robot.
   * @return true, if passed footprint is fully within the footprint of this
   * entity.
   */
  bool contains(const Eigen::Vector2d& pLocation,
                const mars::agent::physical::common::RobotAgentProperties& pRAP);

  bool fitsOn(mars::agent::physical::common::RobotAgentProperties& pRAP);

private:
  static mars::routing::common::Cache<mars::common::Id, mars::routing::common::topology::Vertex>
      sCache;

  std::shared_ptr<mars::routing::common::topology::VertexInterface> mInterface;
  mutable std::shared_ptr<std::recursive_mutex> mMutex;

  std::shared_ptr<std::unordered_map<
      mars::routing::common::topology::ShortestOccupationDurationKey<Edge>, ros::Duration,
      mars::routing::common::topology::ShortestOccupationDurationKey<Edge>::Hash>>
      mShortestOccupationDurations;

  std::shared_ptr<std::unordered_map<
      mars::routing::common::topology::OccupationDurationKey<Edge>, ros::Duration,
      mars::routing::common::topology::OccupationDurationKey<Edge>::Hash>>
      mOccupationDurations;

  std::shared_ptr<int> mType;
  std::shared_ptr<Eigen::Vector3d> mLocation;
  std::shared_ptr<mars::common::geometry::Footprint> mFootprint;
  std::shared_ptr<mars::topology::common::TopologyEntityRestrictions> mRestrictions;

  std::shared_ptr<std::vector<mars::routing::common::topology::Edge>> mIngoingEdges;
  std::shared_ptr<std::vector<mars::routing::common::topology::Edge>> mOutgoingEdges;
};

} // namespace topology
} // namespace common
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_COMMON_TOPOLOGY_VERTEX_H
