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


#ifndef MARS_TOPOLOGY_COMMON_TOPOLOGYENTITY_H
#define MARS_TOPOLOGY_COMMON_TOPOLOGYENTITY_H

#include "TimeIntervalBasedObject.h"
#include "TopologyEntityLock.h"
#include "TopologyEntityReservation.h"
#include "TopologyEntityRestrictions.h"
#include "TopologyEntityType.h"
#include "utility/EntityVisualization.h"

#include <mars_common/Id.h>
#include <mars_common/Logger.h>
#include <mars_common/exception/SetParamException.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#include <boost/container/flat_set.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/circular_buffer.hpp>

namespace mars
{
namespace topology
{
namespace common
{
class TopologyEntity
{
public:
  TopologyEntity(int type, bool isLocked, float entityCoordinateX, float entityCoordinateY,
                 const std::vector<float>& footprintX, const std::vector<float>& footprintY,
                 const std::string& frameId,
                 const mars::topology::common::TopologyEntityRestrictions& pRestrictions);

  TopologyEntity(const std::string id, const std::string& description, int type, bool isLocked,
                 float entityCoordinateX, float entityCoordinateY,
                 const std::vector<float>& footprintX, const std::vector<float>& footprintY,
                 const std::string& frameId,
                 const mars::topology::common::TopologyEntityRestrictions& pRestrictions);

  ~TopologyEntity(){};

  /**
   * @brief implement a function with virtual description.his function is re-defined(Overritten) by
   * a derived class(Edge od Vertex in this case).
   */
  virtual void wtf(void);

  void setCoordinate(const float xPos, const float yPos, const std::string& frameId);

  /**
   * @brief Set the Footprint object
   *
   * @param footprintX
   * @param footprintY
   * @param frameId
   * @throw mars::common::exception::SetParamException
   */
  void setFootprint(const std::vector<float>& footprintX, const std::vector<float>& footprintY,
                    const std::string& frameId) noexcept(false);

  mars::topology::common::TopologyEntityLock
  addLock(const mars::common::Id& initiatorId, const std::string& reason,
          const ros::Time& startTime, const ros::Duration& duration, bool overwriteReservations);

  /**
   * @brief deleteLock Removes a lock from the lock list.
   * @param lockId Id of the lock which should be removed from the lock lists.
   * @return Returns 'true' if a lock can be removed or was removed by another partipant. Returns
   * 'false' if the given id never existed.
   */
  bool deleteLock(const mars::common::Id& lockId, const std::string& reason);

  bool addReservation(const mars::common::Id& agentId, const mars::common::Id& pathId,
                      const mars::common::TimeInterval& timeInterval);

  bool deleteReservation(const mars::common::Id& agentId, const mars::common::Id& pathId);

  mars::common::Id getId(void) const;

  geometry_msgs::PointStamped getCoordinate(void) const;

  geometry_msgs::PolygonStamped getFootprint(void) const;

  std::vector<mars::common::TimeInterval> getFreeTimeSlots(const ros::Time& startTime) const;

  const mars::topology::common::TopologyEntityType& getType(void) const;

  const std::unordered_map<mars::common::Id, mars::topology::common::TopologyEntityLock*>&
  getLocks(void) const;

  const std::unordered_map<std::pair<mars::common::Id, mars::common::Id>,
                           std::vector<mars::topology::common::TopologyEntityReservation*>>&
  getReservations(void) const;

  std::vector<mars::topology::common::TopologyEntityReservation*> getSortedReservations() const;

  const mars::topology::common::TopologyEntityRestrictions& getRestrictions() const;

  mars::topology::common::TopologyEntityReservation* getAllocation() const;

  void setAllocation(mars::topology::common::TopologyEntityReservation* pReservation);

  void drawFootprint(const ros::Publisher& markerPub, std::string frameId,
                     const mars::common::Id& pContainerId);

  bool
  checkReservationAlreadyDeleted(mars::common::Id pAgentId,
                                 std::pair<mars::common::Id, mars::common::Id>& pReservationIdPair);

protected:
  /**
   * @brief mVisualization Helpfer for topology entity visualization.
   */
  mars::topology::common::utility::EntityVisualization mVisualization;

private:
  /**
   * @brief mId The Id of the entity
   */
  mars::common::Id mId;

  /**
   * @brief mTopologyEntityType Type of the topology entity.
   */
  mars::topology::common::TopologyEntityType mTopologyEntityType;

  /**
   * @brief mIsLocked
   */
  bool mIsLocked;

  /**
   * @brief mFootprint The footprint of the entity expressed as a polygon.
   */
  geometry_msgs::PolygonStamped mFootprint;

  /**
   * @brief mPrecisePosition The precise position of the entity.
   */
  geometry_msgs::PointStamped mCoordinate;

  /**
   * @brief mTopologyEntityRestrictions The traversal restrictions of the entity
   */
  mars::topology::common::TopologyEntityRestrictions mTopologyEntityRestrictions;

  std::unordered_map<mars::common::Id, std::string> mDeletedTopologyEntityLocks;

  std::unordered_map<mars::common::Id, mars::topology::common::TopologyEntityLock*>
      mTopologyEntityLocks;
  std::unordered_map<std::pair<mars::common::Id, mars::common::Id>,
                     std::vector<mars::topology::common::TopologyEntityReservation*>>
      mTopologyEntityReservations;

  boost::icl::interval_map<
      ros::Time, boost::container::flat_set<mars::topology::common::TimeIntervalBaseObject*>>
      mIntervalMap;

  mars::topology::common::TopologyEntityReservation* mAllocation;

  boost::circular_buffer<std::pair<mars::common::Id, mars::common::Id>> mRecentDeallocations;

  void setId(const std::string& id, const std::string& description);

  void setStartLock(bool pIsLocked);
};
} // namespace common
} // namespace topology
} // namespace mars

#endif // MARS_TOPOLOGY_COMMON_TOPOLOGYENTITY_H
