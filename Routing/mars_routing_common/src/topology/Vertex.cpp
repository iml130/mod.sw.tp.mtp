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

#include "mars_common/geometry/Segment.h"

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/timeprediction.h"

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <eigen3/Eigen/Core>

#include <tf2_ros/transform_listener.h>

mars::routing::common::topology::Vertex::Vertex(const mars::common::Id& pId, const bool pCache)
    : mars::routing::common::topology::Entity(pId)
{
  if (pCache)
  {
    *this = sCache[pId];
  }
  else
  {
    mMutex = std::make_shared<std::recursive_mutex>();
    mInterface = std::make_shared<mars::routing::common::topology::VertexInterface>(pId);
    mOccupationDurations =
        std::make_shared<std::unordered_map<mars::routing::common::topology::OccupationDurationKey<
                                                mars::routing::common::topology::Edge>,
                                            ros::Duration,
                                            mars::routing::common::topology::OccupationDurationKey<
                                                mars::routing::common::topology::Edge>::Hash>>();
    mShortestOccupationDurations = std::make_shared<
        std::unordered_map<mars::routing::common::topology::ShortestOccupationDurationKey<
                               mars::routing::common::topology::Edge>,
                           ros::Duration,
                           mars::routing::common::topology::ShortestOccupationDurationKey<
                               mars::routing::common::topology::Edge>::Hash>>();
  }
}

int mars::routing::common::topology::Vertex::getType()
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

const boost::optional<Eigen::Vector3d> mars::routing::common::topology::Vertex::getLocation()
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

void mars::routing::common::topology::Vertex::setLocation(const Eigen::Vector3d& pLocation)
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
mars::routing::common::topology::Vertex::getFootprint()
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
mars::routing::common::topology::Vertex::getRestrictions()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (this->mRestrictions == nullptr)
  {
    boost::optional<mars::topology::common::TopologyEntityRestrictions> lRestrictions =
        mInterface->getRestrictions();

    if (!lRestrictions)
    {
      return boost::none;
    }

    this->mRestrictions =
        std::make_shared<mars::topology::common::TopologyEntityRestrictions>(*lRestrictions);
    sCache[mId] = *this;
  }

  return *this->mRestrictions;
}

boost::optional<std::vector<mars::routing::common::topology::Edge>&>
mars::routing::common::topology::Vertex::getIngoingEdges()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};

  *this = sCache[mId];

  if (mIngoingEdges == nullptr)
  {
    mIngoingEdges = std::make_shared<std::vector<mars::routing::common::topology::Edge>>();
    boost::optional<std::vector<mars::common::Id>> lIngoingEdgeIds = mInterface->getIngoingEdges();

    if (!lIngoingEdgeIds)
    {
      return boost::none;
    }

    for (auto& iEdgeId : *lIngoingEdgeIds)
    {
      mIngoingEdges->push_back(mars::routing::common::topology::Edge(iEdgeId));
    }

    sCache[mId] = *this;
  }

  return *mIngoingEdges;
}

boost::optional<std::vector<mars::routing::common::topology::Edge>&>
mars::routing::common::topology::Vertex::getOutgoingEdges()
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  if (mOutgoingEdges == nullptr)
  {
    mOutgoingEdges = std::make_shared<std::vector<mars::routing::common::topology::Edge>>();
    boost::optional<std::vector<mars::common::Id>> lOutgoingEdgeIds =
        mInterface->getOutgoingEdges();

    if (!lOutgoingEdgeIds)
    {
      return boost::none;
    }

    for (auto& iEdgeId : *lOutgoingEdgeIds)
    {
      mOutgoingEdges->push_back(mars::routing::common::topology::Edge(iEdgeId));
    }

    sCache[mId] = *this;
  }

  return *mOutgoingEdges;
}

boost::optional<bool> mars::routing::common::topology::Vertex::isLocked()
{
  // TODO: Status request evalualion
  return false;
}

bool mars::routing::common::topology::Vertex::isTraversable(
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

std::vector<mars::routing::common::topology::Edge>
mars::routing::common::topology::Vertex::getTraversableOutgoingEdges(
    const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  std::vector<mars::routing::common::topology::Edge> lTraversableEdges;

  boost::optional<std::vector<mars::routing::common::topology::Edge>&> lOutgoingEdges =
      this->getOutgoingEdges();

  if (lOutgoingEdges)
  {
    for (auto& iEdge : *lOutgoingEdges)
    {
      if (iEdge.isTraversable(pRAP))
      {
        lTraversableEdges.push_back(iEdge);
      }
    }
  }
  return lTraversableEdges;
}

std::vector<mars::routing::common::topology::Edge>
mars::routing::common::topology::Vertex::getTraversableIngoingEdges(
    const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  std::vector<mars::routing::common::topology::Edge> lTraversableEdges;

  boost::optional<std::vector<mars::routing::common::topology::Edge>&> lIngoingEdges =
      this->getIngoingEdges();

  if (lIngoingEdges)
  {
    for (auto& iEdge : *lIngoingEdges)
    {
      if (iEdge.isTraversable(pRAP))
      {
        lTraversableEdges.push_back(iEdge);
      }
    }
  }

  return lTraversableEdges;
}

std::vector<mars::routing::common::topology::Edge>
mars::routing::common::topology::Vertex::getTraversableTargets(
    const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  return this->getTraversableOutgoingEdges(pRAP);
}

boost::optional<ros::Duration>
mars::routing::common::topology::Vertex::getShortestOccupationDuration(
    mars::routing::common::topology::Edge& pIngoingEdge,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
    mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
        pTimeInfo)
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  mars::routing::common::topology::ShortestOccupationDurationKey<
      mars::routing::common::topology::Edge>
      lKey(pIngoingEdge, pRAP, pVelocity);

  std::vector<mars::routing::common::topology::Edge> lOutgoingEdges =
      this->getTraversableOutgoingEdges(pRAP);

  boost::optional<mars::routing::common::topology::Vertex&> lOrigin = pIngoingEdge.getOrigin(*this);

  if (!lOrigin)
  {
    return boost::none;
  }

  const auto& lShortestOccupationDurationIterator = mShortestOccupationDurations->find(lKey);
  if (lShortestOccupationDurationIterator != mShortestOccupationDurations->end())
  {
    return lShortestOccupationDurationIterator->second;
  }
  else
  {
    ros::Duration lShortestOccupationDuration(mars::common::TimeInterval::INFINITE_DURATION);

    for (auto& iOutgoingEdge : lOutgoingEdges)
    {
      boost::optional<mars::routing::common::topology::Vertex&> lTarget =
          iOutgoingEdge.getTarget(*this);

      if (!lTarget)
      {
        continue;
      }

      boost::optional<ros::Duration> lDuration = mars::routing::common::utility::timeprediction<
          mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
          mars::routing::common::utility::AffineProfile>::getResourceDuration(*this, iOutgoingEdge,
                                                                              pIngoingEdge, pRAP,
                                                                              pVelocity, pTimeInfo);

      if (!lDuration)
      {
        continue;
      }

      if (lShortestOccupationDuration == mars::common::TimeInterval::INFINITE_DURATION ||
          *lDuration < lShortestOccupationDuration)
      {
        lShortestOccupationDuration = *lDuration;
      }
    }

    (*this->mShortestOccupationDurations)[lKey] = lShortestOccupationDuration;
    sCache[mId] = *this;
    return (*this->mShortestOccupationDurations)[lKey];
  }

  return boost::none;
};

boost::optional<ros::Duration> mars::routing::common::topology::Vertex::getOccupationDuration(
    mars::routing::common::topology::Edge& pIngoingEdge,
    mars::routing::common::topology::Edge& pOutgoingEdge,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
    mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
        pTimeInfo)
{
  std::lock_guard<std::recursive_mutex>{*mMutex};
  *this = sCache[mId];

  mars::routing::common::topology::OccupationDurationKey<mars::routing::common::topology::Edge>
      lKey(pIngoingEdge, pOutgoingEdge, pRAP, pVelocity);
  std::vector<mars::routing::common::topology::Edge> lIngoingEdges =
      this->getTraversableIngoingEdges(pRAP);
  std::vector<mars::routing::common::topology::Edge> lOutgoingEdges =
      this->getTraversableOutgoingEdges(pRAP);

  boost::optional<mars::routing::common::topology::Vertex&> lOrigin = pIngoingEdge.getOrigin(*this);
  boost::optional<mars::routing::common::topology::Vertex&> lTarget =
      pOutgoingEdge.getTarget(*this);

  if (!lOrigin || !lTarget)
  {
    return boost::none;
  }

  const auto& lOccupationDurationIterator =
      mOccupationDurations->end(); // mOccupationDurations->find(lKey);
  if (lOccupationDurationIterator != mOccupationDurations->end())
  {
    return lOccupationDurationIterator->second;
  }
  else
  {
    for (auto& iIngoingEdge : lIngoingEdges)
    {
      if (pIngoingEdge != iIngoingEdge)
      {
        continue;
      }

      for (auto& iOutgoingEdge : lOutgoingEdges)
      {
        if (pOutgoingEdge != iOutgoingEdge)
        {
          continue;
        }

        boost::optional<mars::topology::common::TopologyEntityRestrictions&>
            lIngoingEdgeRestrictions = iIngoingEdge.getRestrictions();

        if (!lIngoingEdgeRestrictions)
        {
          continue;
        }

        pVelocity = std::min(pVelocity, lIngoingEdgeRestrictions->getMaxLinearVelocity());

        boost::optional<ros::Duration> lDuration = mars::routing::common::utility::timeprediction<
            mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
            mars::routing::common::utility::AffineProfile>::getResourceDuration(*this,
                                                                                pOutgoingEdge,
                                                                                pIngoingEdge, pRAP,
                                                                                pVelocity,
                                                                                pTimeInfo);

        if (!lDuration)
        {
          continue;
        }

        ros::Duration lOccupationDuration(*lDuration);

        (*this->mOccupationDurations)[lKey] = lOccupationDuration;
        sCache[mId] = *this;
        return (*this->mOccupationDurations)[lKey];
      }
    }
  }

  return boost::none;
}

boost::optional<std::vector<mars::common::TimeInterval>>
mars::routing::common::topology::Vertex::getFreeTimeIntervals(const ros::Time& pStartTime)
{
  return mInterface->getFreeTimeSlots(pStartTime);
}

bool mars::routing::common::topology::Vertex::addReservation(
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

bool mars::routing::common::topology::Vertex::deleteReservation(const mars::common::Id& pAgentId,
                                                                const mars::common::Id& pPathId)
{
  return mInterface->deleteReservation(pAgentId, pPathId).getResultStatus() !=
         mars_common_msgs::Result::RESULT_ERROR;
}

bool mars::routing::common::topology::Vertex::allocate(
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
      this->mInterface->allocateVertex(pAgentId, pPathId, pAllocationInterval, pWaitForGoal,
                                       pDoneCallback, pActiveCallback, pFeedbackCallback);
  sCache[mId] = *this;
  return lSuccess;
}

bool mars::routing::common::topology::Vertex::deallocate(const mars::common::Id& pAgentId,
                                                         const mars::common::Id& pPathId)
{
  mars::common::Result lResult = mInterface->deallocateVertex(pAgentId);
  bool deallocationSuccessful = lResult.getResultStatus() != mars_common_msgs::Result::RESULT_ERROR;

  if (!deallocationSuccessful &&
      ((lResult.getDescription().find(pAgentId.getUUIDAsString(
            mars::common::Id::UUIDFormat::HEXDEC_SPLIT)) != std::string::npos) &&
       (lResult.getDescription().find(pPathId.getUUIDAsString(
            mars::common::Id::UUIDFormat::HEXDEC_SPLIT)) != std::string::npos)))
  {
    deallocationSuccessful = true;

    MARS_LOG_WARN("Found already deallocated entity (vertex) " +
                  pPathId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                  "for agent " +
                  pAgentId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT));
  }

  return deallocationSuccessful;
}

bool mars::routing::common::topology::Vertex::contains(
    const mars::common::geometry::Footprint& pFootprint)
{
  boost::optional<mars::common::geometry::Footprint&> lFootprint = this->getFootprint();

  if (!lFootprint || lFootprint->size() == 0)
  {
    return false;
  }

  return boost::geometry::within(*lFootprint, pFootprint);
}

bool mars::routing::common::topology::Vertex::contains(
    const Eigen::Vector2d& pLocation,
    const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  bool isWithinFootprint = false;
  bool hasEnoughDistance = false;
  double distanceToFootprint = std::numeric_limits<float>::max();
  double robotCollisionRadius;

  boost::optional<mars::common::geometry::Footprint&> lFootprint = this->getFootprint();

  if (!lFootprint || lFootprint->size() == 0)
  {
    return false;
  }

  // distanceToFootprint = boost::geometry::distance(pLocation, *lFootprint);
  robotCollisionRadius = pRAP.getCollisionRadius();

  isWithinFootprint = boost::geometry::within(pLocation, *lFootprint);

  if (isWithinFootprint)
  {
    boost::geometry::for_each_segment(
        *lFootprint, [&distanceToFootprint, &pLocation](const auto& segment) {
          distanceToFootprint =
              std::min<float>(distanceToFootprint, boost::geometry::distance(segment, pLocation));
        });
    hasEnoughDistance = distanceToFootprint > robotCollisionRadius;
  }

  if (isWithinFootprint && hasEnoughDistance)
  {
    return true;
  }

  return false;
}

bool mars::routing::common::topology::Vertex::fitsOn(
    mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  // return this->contains(pRAP.getFootprint());
  return true;
}

mars::routing::common::Cache<mars::common::Id, mars::routing::common::topology::Vertex>
    mars::routing::common::topology::Vertex::sCache;
