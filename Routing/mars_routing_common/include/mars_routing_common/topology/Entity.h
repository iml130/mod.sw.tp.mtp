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

#ifndef MARS_ROUTING_COMMON_TOPOLOGY_ENTITY_H
#define MARS_ROUTING_COMMON_TOPOLOGY_ENTITY_H

#include <actionlib/client/simple_action_client.h>

#include "mars_common/Id.h"

#include "mars_common/geometry/Footprint.h"

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_topology_actions/AllocateEntityAction.h"
#include "mars_topology_common/TopologyEntityRestrictions.h"

#include <boost/optional.hpp>
#include <eigen3/Eigen/Core>

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
/**
 * @class Entity
 */
class Entity
{
public:
  /**
   * @brief Default Entity Constructor with cache.
   * @param pId Unique Id of the vertex. Has to be unique in regards to other
   * topology entities, too.
   * @param pContainerId Unique Id of the container which holds the entity
   * within the topology.
   */
  Entity(const mars::common::Id& pId);

  virtual ~Entity();

  /**
   * @brief Primitive getter for mId.
   * @return Unique Identifier of the entity.
   */
  const mars::common::Id& getId() const;

  /**
   * @brief Complex getter for entity type.
   * May request a connected topology node service, if value is not cached.
   */
  virtual int getType() = 0;

  /**
   * @brief Virtual getter for the location of a topology entity.
   * Not implemented for unspecified entity.
   * May request a connected topology node service, if value is not cached.
   * @return Vector, which points to the location in the world frame.
   * @return boost::none, if service and value are unavailable.
   */
  virtual const boost::optional<Eigen::Vector3d> getLocation() = 0;

  /**
   * @brief Virtual getter for the footprint of a topology entity.
   * Not implemented for unspecified entity.
   * May request a connected topology node service, if value is not cached.
   * @return Footprint in absolute coordinates (world frame).
   * @return boost::none, if service and value are unavailable.
   */
  virtual const boost::optional<mars::common::geometry::Footprint&> getFootprint() = 0;

  /**
   * @brief Virtual getter for the restrictions of a topology entity.
   * Not implemented for unspecified entity.
   * May request a connected topology node service, if value is not cached.
   * @return Restrictions, which need to be met by a robot to be able access
   * this entity.
   * @return boost::none, if service and value are unavailable.
   */
  virtual const boost::optional<mars::topology::common::TopologyEntityRestrictions&>
  getRestrictions() = 0;

  /**
   * @brief Adds a reservation to this entity.
   * @param pAgentId Agent ID of the agent which wants to reservate the entity.
   * @param pPathId Path ID of the current agent path.
   * @param pTimeInterval Interval of the reservation.
   * @return true on success, false otherwise.
   */
  virtual bool addReservation(const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
                              const mars::common::TimeInterval& pTimeInterval) = 0;

  /**
   * @brief Deletes a reservation from this entity.
   * @param pAgentId Agent ID of the agent which causes the deletion.
   * @param pPathId Path ID of the agent path.
   * @return true on success, false otherwise.
   */
  virtual bool deleteReservation(const mars::common::Id& pAgentId,
                                 const mars::common::Id& pPathId) = 0;

  /**
   * @brief TODO: documentation
   */
  virtual bool
  allocate(const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
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
                   mars_topology_actions::AllocateEntityAction>::SimpleFeedbackCallback()) = 0;

  /**
   * @brief Frees this entity, in order to make it usable by other agents.
   * @param pAgentId Agent ID of the agent which shall "do" the deallocation.
   * @return true on success, false otherwise.
   */
  virtual bool deallocate(const mars::common::Id& pAgentId, const mars::common::Id& pPathId) = 0;

  /**
   * @brief Checks if the passed footprint is fully inside this entitie's
   * footprint.
   * @param pFootprint Footprint to be checked.
   * @return true, if passed footprint is fully within the footprint of this
   * entity.
   */
  virtual bool contains(const mars::common::geometry::Footprint& pFootprint) = 0;

  /**
   * @brief Checks if the passed footprint is fully inside this entitie's
   * footprint.
   * @param pLocation Location of the robot.
   * @param pRAP Properties of the robot.
   * @return true, if passed footprint is fully within the footprint of this
   * entity.
   */
  virtual bool contains(const Eigen::Vector2d& pLocation,
                        const mars::agent::physical::common::RobotAgentProperties& pRAP) = 0;

  virtual bool fitsOn(mars::agent::physical::common::RobotAgentProperties& pRAP) = 0;

  /**
   * @brief Simple comparator between two generic entities of any type.
   * @return True, if this and the compared entity are the same in the topology.
   */
  bool operator==(const Entity& pComparedEntity) const;

  /**
   * @brief Simple comparator between two generic entities of any type.
   * @return True, if this and the compared entity are different entities in the
   * topology.
   */
  bool operator!=(const Entity& pComparedEntity) const;

protected:
  mars::common::Id mId;
};
} // namespace topology
} // namespace common
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_COMMON_TOPOLOGY_ENTITY_H
