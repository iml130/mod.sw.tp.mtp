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

#include "mars_routing_common/topology/Edge.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/timeprediction.h"

#include <tf2_ros/transform_listener.h>

mars::routing::common::topology::Edge::Edge(const mars::common::Id& pId, const bool pCache)
    : mars::routing::common::topology::Entity(pId)
{
  if (pCache)
  {
    *this = sCache[pId];
  }
  else
  {
    mMutex = std::make_shared<std::recursive_mutex>();
    mInterface = std::make_shared<mars::routing::common::topology::EdgeInterface>(pId);
    mShortestOccupationDurations = std::make_shared<
        std::unordered_map<mars::routing::common::topology::ShortestOccupationDurationKey<
                               mars::routing::common::topology::Vertex>,
                           ros::Duration,
                           mars::routing::common::topology::ShortestOccupationDurationKey<
                               mars::routing::common::topology::Vertex>::Hash>>();
  }
}

int mars::routing::common::topology::Edge::getType()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mType == nullptr)
  {
    boost::optional<int> lType = mInterface->getType();

    if (!lType)
    {
      return mars_topology_msgs::TopologyEntityType::TOPOLOGY_ENTITY_TYPE_UNKNOWN;
    }

    mType = std::make_shared<int>(*lType);

    sCache[mId] = *this;
  }

  return *mType;
}

const boost::optional<Eigen::Vector3d> mars::routing::common::topology::Edge::getLocation()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mLocation == nullptr)
  {
    boost::optional<Eigen::Vector3d> lLocation = mInterface->getCoordinate();

    if (!lLocation)
    {
      return boost::none;
    }

    mLocation = std::make_shared<Eigen::Vector3d>(*lLocation);

    sCache[mId] = *this;
  }

  return *mLocation;
}

void mars::routing::common::topology::Edge::setLocation(const Eigen::Vector3d& pLocation)
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mLocation == nullptr)
  {
    mLocation = std::make_shared<Eigen::Vector3d>(pLocation);

    sCache[mId] = *this;
  }
}

const boost::optional<mars::common::geometry::Footprint&>
mars::routing::common::topology::Edge::getFootprint()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mFootprint == nullptr)
  {
    boost::optional<mars::common::geometry::Footprint> lFootprint = mInterface->getFootprint();

    if (!lFootprint)
    {
      return boost::none;
    }

    mFootprint = std::make_shared<mars::common::geometry::Footprint>(*lFootprint);

    sCache[mId] = *this;
  }

  return *mFootprint;
}

const boost::optional<mars::topology::common::TopologyEntityRestrictions&>
mars::routing::common::topology::Edge::getRestrictions()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mRestrictions == nullptr)
  {
    boost::optional<mars::topology::common::TopologyEntityRestrictions> lRestrictions =
        mInterface->getRestrictions();

    if (!lRestrictions)
    {
      return boost::none;
    }

    mRestrictions =
        std::make_shared<mars::topology::common::TopologyEntityRestrictions>(*lRestrictions);

    sCache[mId] = *this;
  }

  return *mRestrictions;
}

boost::optional<double> mars::routing::common::topology::Edge::getLength()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mLength == nullptr)
  {
    boost::optional<float> lLength = mInterface->getLength();

    if (!lLength)
    {
      return boost::none;
    }

    mLength = std::make_shared<double>(*lLength);

    sCache[mId] = *this;
  }

  return *mLength;
}

boost::optional<mars::routing::common::topology::Vertex&>
mars::routing::common::topology::Edge::getOrigin(mars::routing::common::topology::Vertex& pTarget)
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mConnections == nullptr)
  {
    boost::optional<std::vector<mars::routing::common::topology::Connection>> lEdgeConnections =
        mInterface->getConnections();

    if (!lEdgeConnections)
    {
      return boost::none;
    }

    std::vector<mars::routing::common::topology::Connection> lConnections(*lEdgeConnections);
    mConnections =
        std::make_shared<std::vector<mars::routing::common::topology::Connection>>(lConnections);

    sCache[mId] = *this;
  }

  for (auto& iConnection : *mConnections)
  {
    if (iConnection.getDestination() == pTarget)
    {
      return iConnection.getOrigin();
    }
  }

  return boost::none;
}

boost::optional<mars::routing::common::topology::Vertex&>
mars::routing::common::topology::Edge::getTarget(mars::routing::common::topology::Vertex& pOrigin)
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mConnections == nullptr)
  {
    boost::optional<std::vector<mars::routing::common::topology::Connection>> lEdgeConnections =
        mInterface->getConnections();

    if (!lEdgeConnections)
    {
      return boost::none;
    }

    std::vector<mars::routing::common::topology::Connection> lConnections(*lEdgeConnections);
    mConnections =
        std::make_shared<std::vector<mars::routing::common::topology::Connection>>(lConnections);

    sCache[mId] = *this;
  }

  for (auto& iConnection : *mConnections)
  {
    if (iConnection.getOrigin() == pOrigin)
    {
      return iConnection.getDestination();
    }
  }

  return boost::none;
}

std::vector<mars::routing::common::topology::Vertex>
mars::routing::common::topology::Edge::getTraversableTargets(
    const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  std::vector<mars::routing::common::topology::Vertex> lTraversableTargets;

  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mConnections == nullptr)
  {
    boost::optional<std::vector<mars::routing::common::topology::Connection>> lEdgeConnections =
        mInterface->getConnections();

    if (!lEdgeConnections)
    {
      return lTraversableTargets;
    }

    std::vector<mars::routing::common::topology::Connection> lConnections(*lEdgeConnections);
    mConnections =
        std::make_shared<std::vector<mars::routing::common::topology::Connection>>(lConnections);

    sCache[mId] = *this;
  }

  for (auto& iConnection : *mConnections)
  {
    mars::routing::common::topology::Vertex& lTarget = iConnection.getDestination();
    if (lTarget.isTraversable(pRAP))
    {
      lTraversableTargets.push_back(lTarget);
    }
  }

  return lTraversableTargets;
}

boost::optional<bool> mars::routing::common::topology::Edge::isLocked()
{
  // TODO: Status request evaluation
  return false;
}

bool mars::routing::common::topology::Edge::isTraversable(
    const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  boost::optional<bool> lIsLocked = this->isLocked();
  const boost::optional<mars::topology::common::TopologyEntityRestrictions&> lRestrictions =
      this->getRestrictions();

  if (lRestrictions && lIsLocked)
  {
    return pRAP.match(*lRestrictions) && !*lIsLocked;
  }

  return false;
}

boost::optional<std::vector<mars::common::TimeInterval>>
mars::routing::common::topology::Edge::getFreeTimeIntervals(const ros::Time& pStartTime)
{
  return mInterface->getFreeTimeSlots(pStartTime);
}

boost::optional<ros::Duration> mars::routing::common::topology::Edge::getShortestOccupationDuration(
    mars::routing::common::topology::Vertex& pEntryVertex,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
    mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
        pTimeInfo)
{
  boost::optional<mars::routing::common::topology::Vertex&> lExitVertex =
      this->getTarget(pEntryVertex);

  if (!lExitVertex)
  {
    return boost::none;
  }

  return this->getOccupationDuration(pEntryVertex, *lExitVertex, pRAP, pVelocity, pTimeInfo);
}

boost::optional<ros::Duration> mars::routing::common::topology::Edge::getOccupationDuration(
    mars::routing::common::topology::Vertex& pEntryVertex,
    mars::routing::common::topology::Vertex& pExitVertex,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
    mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
        pTimeInfo)
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  mars::routing::common::topology::ShortestOccupationDurationKey<
      mars::routing::common::topology::Vertex>
      lKey(pEntryVertex, pRAP, pVelocity);

  const auto& lShortestOccupationDurationIterator =
      mShortestOccupationDurations->end(); // mShortestOccupationDurations->find(lKey);
  if (lShortestOccupationDurationIterator != mShortestOccupationDurations->end())
  {
    return lShortestOccupationDurationIterator->second;
  }
  else
  {
    // Check if ingoing or outgoing vertex is not connected to this edge
    bool lEntryFound = false;
    bool lExitFound = false;
    boost::optional<std::vector<mars::routing::common::topology::Edge>&> lEntryOutgoingEdges =
        pEntryVertex.getOutgoingEdges();
    boost::optional<std::vector<mars::routing::common::topology::Edge>&> lExitOutgoingEdges =
        pExitVertex.getIngoingEdges();

    if (!lEntryOutgoingEdges || !lExitOutgoingEdges)
    {
      return boost::none;
    }

    for (mars::routing::common::topology::Edge i_Edge : *lEntryOutgoingEdges)
    {
      if (i_Edge == *this)
      {
        lEntryFound = true;
        break;
      }
    }

    for (mars::routing::common::topology::Edge i_Edge : *lExitOutgoingEdges)
    {
      if (i_Edge == *this)
      {
        lExitFound = true;
        break;
      }
    }

    if (!lEntryFound || !lExitFound)
    {
      return boost::none;
    }

    boost::optional<ros::Duration> lDuration = mars::routing::common::utility::timeprediction<
        mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
        mars::routing::common::utility::AffineProfile>::getResourceDuration(*this, pExitVertex,
                                                                            pEntryVertex, pRAP,
                                                                            pVelocity, pTimeInfo);

    if (!lDuration)
    {
      return boost::none;
    }

    (*this->mShortestOccupationDurations)[lKey] = *lDuration;
    sCache[mId] = *this;
    return (*this->mShortestOccupationDurations)[lKey];
  }

  return boost::none;
}

bool mars::routing::common::topology::Edge::addReservation(
    const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
    const mars::common::TimeInterval& pTimeInterval)
{
  mars::common::Result ReservationResult =
      mInterface->addReservation(pAgentId, pPathId, pTimeInterval);

  if (ReservationResult.getResultStatus() == mars_common_msgs::Result::RESULT_ERROR)
  {
    MARS_LOG_ERROR("An error occurred while adding a reservation to an topology entity. \n"
                   << ReservationResult.getDescription());
    return false;
  }
  else
  {
    return true;
  }

  return ReservationResult.getResultStatus() != mars_common_msgs::Result::RESULT_ERROR;
}

bool mars::routing::common::topology::Edge::deleteReservation(const mars::common::Id& pAgentId,
                                                              const mars::common::Id& pPathId)
{
  return mInterface->deleteReservation(pAgentId, pPathId).getResultStatus() !=
         mars_common_msgs::Result::RESULT_ERROR;
}

bool mars::routing::common::topology::Edge::allocate(
    const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
    const mars::common::TimeInterval& pAllocationInterval, const ros::Duration& pWaitForGoal,
    const actionlib::SimpleActionClient<
        mars_topology_actions::AllocateEntityAction>::SimpleDoneCallback& pDoneCallback,
    const actionlib::SimpleActionClient<
        mars_topology_actions::AllocateEntityAction>::SimpleActiveCallback& pActiveCallback,
    const actionlib::SimpleActionClient<
        mars_topology_actions::AllocateEntityAction>::SimpleFeedbackCallback& pFeedbackCallback)
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  bool lSuccess =
      this->mInterface->allocateEdge(pAgentId, pPathId, pAllocationInterval, pWaitForGoal,
                                     pDoneCallback, pActiveCallback, pFeedbackCallback);
  sCache[mId] = *this;
  return lSuccess;
}

bool mars::routing::common::topology::Edge::deallocate(const mars::common::Id& pAgentId, const mars::common::Id& pPathId)
{
  mars::common::Result lResult = mInterface->deallocateEdge(pAgentId);
  bool deallocationSuccessful = lResult.getResultStatus() != mars_common_msgs::Result::RESULT_ERROR;

  if (!deallocationSuccessful &&
      ((lResult.getDescription().find(pAgentId.getUUIDAsString(
            mars::common::Id::UUIDFormat::HEXDEC_SPLIT)) != std::string::npos) &&
       (lResult.getDescription().find(pPathId.getUUIDAsString(
            mars::common::Id::UUIDFormat::HEXDEC_SPLIT)) != std::string::npos)))
  {
    deallocationSuccessful = true;

    MARS_LOG_WARN("Found already deallocated entity (edge) " +
                  pPathId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                  "for agent " +
                  pAgentId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT));
  }

  return deallocationSuccessful;
}

bool mars::routing::common::topology::Edge::contains(
    const mars::common::geometry::Footprint& pFootprint)
{
  boost::optional<mars::common::geometry::Footprint&> lFootprint = this->getFootprint();

  if (!lFootprint || lFootprint->size() == 0)
  {
    return false;
  }

  return boost::geometry::within(*lFootprint, pFootprint);
}

bool mars::routing::common::topology::Edge::contains(
    const Eigen::Vector2d& pLocation,
    const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  boost::optional<mars::common::geometry::Footprint&> lFootprint = this->getFootprint();

  if (!lFootprint || lFootprint->size() == 0)
  {
    return false;
  }

  double lDistanceToFootprint = boost::geometry::distance(pLocation, *lFootprint);

  if (boost::geometry::within(pLocation, *lFootprint) &&
      lDistanceToFootprint > pRAP.getCollisionRadius())
  {
    return true;
  }

  return false;
}

bool mars::routing::common::topology::Edge::fitsOn(
    mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  boost::optional<double> length;
  do
  {
    length = *this->getLength();

  } while (!length);

  return (*length / 2.0) >= pRAP.getCollisionRadius();
}

mars::routing::common::Cache<mars::common::Id, mars::routing::common::topology::Edge>
    mars::routing::common::topology::Edge::sCache;
