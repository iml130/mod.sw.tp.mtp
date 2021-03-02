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


#include "mars_topology_common/TopologyEntity.h"

#include <algorithm>
#include <ros/console.h>

static const std::string ENTITY_NAMESPACE = "mars_entity";
static const std::string REASON_LOCKED_FROM_START = "Locked from start";

static const std::string OVERWRITE_RESERVATIONS_NOT_SUPPORTED = "Overwrite reservations to set a "
                                                                "lock is currently not "
                                                                "supported!";

mars::topology::common::TopologyEntity::TopologyEntity(
    int type, bool isLocked, float entityCoordinateX, float entityCoordinateY,
    const std::vector<float>& footprintX, const std::vector<float>& footprintY,
    const std::string& frameId,
    const mars::topology::common::TopologyEntityRestrictions& pRestrictions)
    : mTopologyEntityType(type), mTopologyEntityRestrictions(pRestrictions), mAllocation(nullptr)
{
  this->mId.initialize();
  this->setCoordinate(entityCoordinateX, entityCoordinateY, frameId);
  this->setFootprint(footprintX, footprintY, frameId);

  this->setStartLock(isLocked);
  this->mRecentDeallocations.set_capacity(100);
}

mars::topology::common::TopologyEntity::TopologyEntity(
    const std::string id, const std::string& description, int type, bool isLocked,
    float entityCoordinateX, float entityCoordinateY, const std::vector<float>& footprintX,
    const std::vector<float>& footprintY, const std::string& frameId,
    const mars::topology::common::TopologyEntityRestrictions& pRestrictions)
    : mTopologyEntityType(type), mTopologyEntityRestrictions(pRestrictions), mAllocation(nullptr)
{
  this->setId(id, description);
  this->setCoordinate(entityCoordinateX, entityCoordinateY, frameId);
  this->setFootprint(footprintX, footprintY, frameId);

  this->setStartLock(isLocked);
  this->mRecentDeallocations.set_capacity(100);
}

void mars::topology::common::TopologyEntity::setId(const std::string& id,
                                                   const std::string& description)
{
  this->mId.initialize(id, description);
}

void mars::topology::common::TopologyEntity::setStartLock(bool pIsLocked)
{
  if (pIsLocked)
  {
    this->mIsLocked = true;

    this->addLock(this->mId, REASON_LOCKED_FROM_START, ros::Time::now(),
                  mars::common::TimeInterval::INFINITE_DURATION, false);
  }
  else
  {
    this->mIsLocked = false;
  }
}

void mars::topology::common::TopologyEntity::drawFootprint(const ros::Publisher& markerPub,
                                                           std::string frameId,
                                                           const mars::common::Id& pContainerId)
{
  this->mVisualization.createMarkerLineStrip(this->mFootprint, frameId, ENTITY_NAMESPACE);
}

void mars::topology::common::TopologyEntity::setCoordinate(const float xPos, const float yPos,
                                                           const std::string& frameId)
{
  this->mCoordinate.header.stamp = ros::Time::now();
  this->mCoordinate.header.frame_id = frameId;
  this->mCoordinate.point.x = xPos;
  this->mCoordinate.point.y = yPos;
  this->mCoordinate.point.z = 0;
}

void mars::topology::common::TopologyEntity::setFootprint(
    const std::vector<float>& footprintX, const std::vector<float>& footprintY,
    const std::string& frameId) noexcept(false)
{
  if (footprintX.size() == footprintY.size())
  {
    for (std::size_t i = 0; i < footprintX.size(); ++i)
    {
      geometry_msgs::Point32 tmpPoint;

      tmpPoint.x = footprintX[i];
      tmpPoint.y = footprintY[i];
      tmpPoint.z = 0.0;

      this->mFootprint.polygon.points.push_back(tmpPoint);
    }

    this->mFootprint.header.frame_id = frameId;
  }
  else
  {
    throw mars::common::exception::SetParamException("Can't set footprint, x-coordinates and "
                                                     "y-coordinates do not have "
                                                     "the same length!");
  }
}

mars::topology::common::TopologyEntityLock mars::topology::common::TopologyEntity::addLock(
    const mars::common::Id& initiatorId, const std::string& reason, const ros::Time& startTime,
    const ros::Duration& duration, bool overwriteReservations)
{
  mars::topology::common::TopologyEntityLock* topologyEntityLock =
      new mars::topology::common::TopologyEntityLock(initiatorId, reason, startTime, duration);

  // the overwrite of reservation will be supported in MARS version V2 or above.
  if (overwriteReservations)
  {
    ROS_WARN_STREAM("[mars::topology::common::TopologyEntity::addLock]"
                    << OVERWRITE_RESERVATIONS_NOT_SUPPORTED);
  }

  this->mTopologyEntityLocks.emplace(topologyEntityLock->getId(), topologyEntityLock);
  this->mIntervalMap +=
      std::make_pair(boost::icl::interval<ros::Time>::right_open(
                         topologyEntityLock->getTimeInterval().getStartTime(),
                         topologyEntityLock->getTimeInterval().getEndTime()),
                     boost::container::flat_set<mars::topology::common::TimeIntervalBaseObject*>{
                         topologyEntityLock});

  ROS_DEBUG_STREAM("[mars::topology::common::TopologyEntity::addLock] Added new lock: "
                   << std::endl
                   << std::to_string(*topologyEntityLock));

  return *topologyEntityLock;
}

bool mars::topology::common::TopologyEntity::deleteLock(const mars::common::Id& lockId,
                                                        const std::string& reason)
{
  bool removedLock = true;

  const auto& deletedLock = this->mTopologyEntityLocks.find(lockId);

  if (deletedLock != this->mTopologyEntityLocks.end())
  {
    this->mDeletedTopologyEntityLocks.insert(
        std::pair<mars::common::Id, std::string>(deletedLock->second->getId(), reason));

    this->mIntervalMap.erase(
        std::make_pair(boost::icl::continuous_interval<ros::Time>(
                           deletedLock->second->getTimeInterval().getStartTime(),
                           deletedLock->second->getTimeInterval().getEndTime()),
                       boost::container::flat_set<mars::topology::common::TimeIntervalBaseObject*>{
                           deletedLock->second}));

    delete deletedLock->second;
    this->mTopologyEntityLocks.erase(deletedLock);

    ROS_DEBUG_STREAM("[mars::topology::common::TopologyEntity::deleteLock] "
                     "Successfully removed lock: "
                     << std::endl
                     << std::to_string(*deletedLock->second));
  }
  else if (this->mDeletedTopologyEntityLocks.find(lockId) !=
           this->mDeletedTopologyEntityLocks.end())
  {
    const auto& it = this->mDeletedTopologyEntityLocks.find(lockId);

    ROS_DEBUG_STREAM("[mars::topology::common::TopologyEntity::deleteLock] Lock "
                     "was already removed: "
                     << std::endl
                     << std::to_string(it->first) << std::endl
                     << "Reason: " << it->second);
  }
  else
  {
    ROS_ERROR_STREAM("[mars::topology::common::TopologyEntity::deleteLock] "
                     "Can't remove lock, lock "
                     "with id doesn't exists: "
                     << std::endl
                     << std::to_string(lockId));

    removedLock = false;
  }

  return removedLock;
}

bool mars::topology::common::TopologyEntity::addReservation(
    const mars::common::Id& agentId, const mars::common::Id& pathId,
    const mars::common::TimeInterval& timeInterval)
{
  bool successfullyAddedReservation = false;

  mars::topology::common::TopologyEntityReservation* topologyEntityReservation =
      new mars::topology::common::TopologyEntityReservation(agentId, pathId, timeInterval);

  boost::icl::continuous_interval<ros::Time> lReservationBoostInterval(timeInterval.getStartTime(),
                                                                       timeInterval.getEndTime());

  auto lReservationPair =
      std::make_pair(lReservationBoostInterval,
                     boost::container::flat_set<mars::topology::common::TimeIntervalBaseObject*>{
                         topologyEntityReservation});

  auto intersections = mIntervalMap.equal_range(lReservationBoostInterval);

  if (intersections.first == intersections.second)
  {
    auto lReservations = this->mTopologyEntityReservations.find(std::make_pair(agentId, pathId));

    std::vector<mars::topology::common::TopologyEntityReservation*> lExistingReservations =
        this->getSortedReservations();
    bool lReservationValid = false;

    if (lExistingReservations.size() > 0)
    {
      if (*lExistingReservations[0] <= *topologyEntityReservation)
      {
        lReservationValid = true;
      }
      else if ((*topologyEntityReservation <= *lExistingReservations[0]) &&
               !(lExistingReservations[0] == this->getAllocation()))
      {
        lReservationValid = true;
      }
      else
      {
        lReservationValid = false;
      }
    }
    else
    {
      lReservationValid = true;
    }

    if (lReservationValid)
    {
      if (lReservations == this->mTopologyEntityReservations.end())
      {
        // No reservation for given pair of agent and route on this entity yet
        auto lNewReservationResult = this->mTopologyEntityReservations.emplace(
            std::make_pair(agentId, pathId),
            std::vector<mars::topology::common::TopologyEntityReservation*>{
                topologyEntityReservation});
        if (!lNewReservationResult.second)
        {
          // This should never happen, but just to be sure:
          ROS_ERROR(
              "[mars::topology::common::TopologyEntity::addReservation] Can't add reservation");
        }
        else
        {
          this->mIntervalMap += lReservationPair;
          successfullyAddedReservation = true;
        }
      }
      else
      {
        // There is at least one reservation for this agent and route already
        lReservations->second.push_back(topologyEntityReservation);
        this->mIntervalMap += lReservationPair;
        successfullyAddedReservation = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("[mars::topology::common::TopologyEntity::addReservation] Can't add "
                       "reservation before allocated reservation!");
    }
  }
  else
  {
    delete topologyEntityReservation;
    ROS_DEBUG_STREAM("[mars::topology::common::TopologyEntity::addReservation] Can't add "
                     "reservation: Interval is "
                     "locked or already allocated!");
  }

  return successfullyAddedReservation;
}

bool mars::topology::common::TopologyEntity::deleteReservation(const mars::common::Id& agentId,
                                                               const mars::common::Id& pathId)
{
  bool reservationDeleted = false;

  const auto& lReservationToDelete =
      this->mTopologyEntityReservations.find(std::make_pair(agentId, pathId));

  if (lReservationToDelete != this->mTopologyEntityReservations.end())
  {
    // Pair of agent and path has at least one reservation on this entity

    for (mars::topology::common::TopologyEntityReservation* iReservation :
         lReservationToDelete->second)
    {

      this->mIntervalMap.erase(std::make_pair(
          boost::icl::continuous_interval<ros::Time>(iReservation->getTimeInterval().getStartTime(),
                                                     iReservation->getTimeInterval().getEndTime()),
          boost::container::flat_set<mars::topology::common::TimeIntervalBaseObject*>{
              iReservation}));

      delete iReservation;
    }
    this->mTopologyEntityReservations.erase(lReservationToDelete);
    this->mRecentDeallocations.push_front(std::pair<mars::common::Id, mars::common::Id>(agentId, pathId));
    reservationDeleted = true;
  }

  return reservationDeleted;
}

mars::common::Id mars::topology::common::TopologyEntity::getId() const { return this->mId; }

geometry_msgs::PointStamped mars::topology::common::TopologyEntity::getCoordinate(void) const
{
  return this->mCoordinate;
}

geometry_msgs::PolygonStamped mars::topology::common::TopologyEntity::getFootprint() const
{
  return this->mFootprint;
}

std::vector<mars::common::TimeInterval>
mars::topology::common::TopologyEntity::getFreeTimeSlots(const ros::Time& startTime) const
{
  // stores found free time slots
  std::vector<mars::common::TimeInterval> lFreeTimeSlots;

  ros::Time lStartTime = startTime;

  unsigned int numberOfReservations = this->mIntervalMap.iterative_size();

  // set up iterator
  auto lCurrentBoostInterval = this->mIntervalMap.begin();
  auto lNextBoostInterval = this->mIntervalMap.begin();

  switch (numberOfReservations)
  {
  // no reservations available
  case 0:
    // add one free time slot of infinite length
    lFreeTimeSlots.push_back(
        mars::common::TimeInterval(ros::Time(0), mars::common::TimeInterval::INFINITE_DURATION));
    break;

  // Only one reservation available
  case 1:

    lNextBoostInterval++;
    // check if it was already allocated
    if (this->mAllocation != nullptr)
    {
      // add free time slot of infinte length after the allocated reservation
      lFreeTimeSlots.push_back(mars::common::TimeInterval(
          lCurrentBoostInterval->first.upper(), mars::common::TimeInterval::INFINITE_DURATION));
    }
    else
    {
      // check if there is time before the reservation
      if (lCurrentBoostInterval->first.lower() > lStartTime)
      {
        // add one slot before the reservation and an infinite one after
        lFreeTimeSlots.push_back(mars::common::TimeInterval(
            lStartTime, lCurrentBoostInterval->first.lower() - lStartTime));

        lFreeTimeSlots.push_back(mars::common::TimeInterval(
            lCurrentBoostInterval->first.upper(), mars::common::TimeInterval::INFINITE_DURATION));
      }
      else
      {
        // add free time slot of infinte length after the reservation
        lFreeTimeSlots.push_back(mars::common::TimeInterval(
            lCurrentBoostInterval->first.upper(), mars::common::TimeInterval::INFINITE_DURATION));
      }
    }
    break;
  // there are more reservations available than the two special cases
  default:
    lNextBoostInterval++;
    // check if the first reservation is allocated
    if (this->mAllocation != nullptr) // TODO check if allocation and first reservation are the same
    {
      lStartTime = ros::Time(lCurrentBoostInterval->first.upper());
    }
    // not allocated
    else
    {
      // check if there is time before the reservation
      if (lCurrentBoostInterval->first.lower() > lStartTime)
      {
        // add one slot before the reservation and an infinite one after
        lFreeTimeSlots.push_back(mars::common::TimeInterval(
            lStartTime, lCurrentBoostInterval->first.lower() - lStartTime));
      }
    }
    while (lNextBoostInterval != this->mIntervalMap.end())
    {
      if (lNextBoostInterval->first.lower() > lStartTime)
      {

        ros::Duration duration(lNextBoostInterval->first.lower() -
                               lCurrentBoostInterval->first.upper());

        if (duration > ros::Duration(0))
        {
          lFreeTimeSlots.push_back(
              mars::common::TimeInterval(lCurrentBoostInterval->first.upper(), duration));
        }
      }

      lCurrentBoostInterval++;
      lNextBoostInterval++;
    }

    // Check if last available reservation is infinite
    if (lCurrentBoostInterval->first.upper() != mars::common::TimeInterval::INFINITE_END_TIME)
    {
      lFreeTimeSlots.push_back(mars::common::TimeInterval(
          lCurrentBoostInterval->first.upper(), mars::common::TimeInterval::INFINITE_DURATION));
    }

    break;
  }
  return lFreeTimeSlots;
}

const mars::topology::common::TopologyEntityType&
mars::topology::common::TopologyEntity::getType() const
{
  return this->mTopologyEntityType;
}

const std::unordered_map<mars::common::Id, mars::topology::common::TopologyEntityLock*>&
mars::topology::common::TopologyEntity::getLocks() const
{
  return this->mTopologyEntityLocks;
}

const std::unordered_map<std::pair<mars::common::Id, mars::common::Id>,
                         std::vector<mars::topology::common::TopologyEntityReservation*>>&
mars::topology::common::TopologyEntity::getReservations() const
{
  return this->mTopologyEntityReservations;
}

std::vector<mars::topology::common::TopologyEntityReservation*>
mars::topology::common::TopologyEntity::getSortedReservations() const
{
  std::vector<mars::topology::common::TopologyEntityReservation*> lReservations(0);

  for (const auto& iReservations : this->mTopologyEntityReservations)
  {
    for (mars::topology::common::TopologyEntityReservation* iReservation : iReservations.second)
      lReservations.push_back(iReservation);
  }

  std::sort(
      lReservations.begin(), lReservations.end(),
      [](mars::topology::common::TopologyEntityReservation* pFront,
         mars::topology::common::TopologyEntityReservation* pBack) { return *pFront < *pBack; });

  return lReservations;
}

const mars::topology::common::TopologyEntityRestrictions&
mars::topology::common::TopologyEntity::getRestrictions() const
{
  return this->mTopologyEntityRestrictions;
}

mars::topology::common::TopologyEntityReservation*
mars::topology::common::TopologyEntity::getAllocation() const
{
  return this->mAllocation;
}

void mars::topology::common::TopologyEntity::setAllocation(
    mars::topology::common::TopologyEntityReservation* pReservation)
{
  this->mAllocation = pReservation;
}
// the virtual function is defined here
void mars::topology::common::TopologyEntity::wtf(void) {}

bool mars::topology::common::TopologyEntity::checkReservationAlreadyDeleted(
    mars::common::Id pAgentId, std::pair<mars::common::Id, mars::common::Id>& pReservationIdPair)
{
  bool lFoundReservationEntry = false;

  for (size_t i = 0; i < this->mRecentDeallocations.size(); i++)
  {
    std::pair<mars::common::Id, mars::common::Id> tmpReservation = this->mRecentDeallocations[i];

    if (tmpReservation.first == pAgentId)
    {
      pReservationIdPair = tmpReservation;
      lFoundReservationEntry = true;
      break;
    }
  }

  return lFoundReservationEntry;
}
