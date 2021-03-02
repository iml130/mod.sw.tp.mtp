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

#include "mars_routing_core/Step.h"

#include "mars_common/geometry/Segment.h"
#include "mars_routing_common/geometry/utilities.h"

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_common/utility/AffineProfile.h"

#include <boost/geometry/algorithms/intersection.hpp>
#include <eigen_conversions/eigen_msg.h>

template <class Origin, class Target>
mars::routing::core::Step<Origin, Target>::Step(
    const Origin& pOrigin, const Target& pTarget,
    const mars::common::TimeInterval& pTargetOccupationInterval,
    mars::routing::core::Step<Target, Origin>* pPrevious,
    mars::routing::core::Step<Target, Origin>* pNext)
    : mOrigin(pOrigin), mTarget(pTarget), mTargetOccupationInterval(pTargetOccupationInterval),
      mPrevious(pPrevious), mNext(pNext)
{
}

template <class Origin, class Target>
mars::routing::core::Step<Origin, Target>::Step(const mars_routing_msgs::Step& pStepMsg)
    : mOrigin(mars::common::Id(pStepMsg.origin.id)),
      mTarget(mars::common::Id(pStepMsg.destination.id)),
      mTargetOccupationInterval(pStepMsg.time_interval), mPrevious(nullptr), mNext(nullptr)
{
  Eigen::Vector3d lOriginLocation;
  Eigen::Vector3d lTargetLocation;

  tf::pointMsgToEigen(pStepMsg.origin_location.point, lOriginLocation);
  tf::pointMsgToEigen(pStepMsg.destination_location.point, lTargetLocation);

  mOrigin.setLocation(lOriginLocation);
  mTarget.setLocation(lTargetLocation);
}

template <class Origin, class Target> mars::routing::core::Step<Origin, Target>::~Step() {}

template <class Origin, class Target>
mars::routing::core::Step<Origin, Target>* mars::routing::core::Step<Origin, Target>::clone() const
{
  return new mars::routing::core::Step<Origin, Target>(
      this->mOrigin, this->mTarget, this->mTargetOccupationInterval, this->mPrevious, this->mNext);
}

template <class Origin, class Target> Origin& mars::routing::core::Step<Origin, Target>::getOrigin()
{
  return this->mOrigin;
}

template <class Origin, class Target> Target& mars::routing::core::Step<Origin, Target>::getTarget()
{
  return this->mTarget;
}

template <class Origin, class Target>
double mars::routing::core::Step<Origin, Target>::getDistance()
{
  const boost::optional<Eigen::Vector3d> lOriginLocation = mOrigin.getLocation();
  const boost::optional<Eigen::Vector3d> lTargetLocation = mTarget.getLocation();

  if (!lOriginLocation || !lTargetLocation)
  {
    // TODO: Handle location request failures!
    return 0;
  }

  return (*lOriginLocation - *lTargetLocation).norm();
}

template <class Origin, class Target>
const mars::common::TimeInterval&
mars::routing::core::Step<Origin, Target>::getTargetOccupationInterval() const
{
  return this->mTargetOccupationInterval;
}

template <class Origin, class Target>
mars::routing::core::Step<Target, Origin>*
mars::routing::core::Step<Origin, Target>::getPrevious() const
{
  return this->mPrevious;
}

template <class Origin, class Target>
mars::routing::core::Step<Target, Origin>*
mars::routing::core::Step<Origin, Target>::getNext() const
{
  return this->mNext;
}

template <class Origin, class Target>
void mars::routing::core::Step<Origin, Target>::setTargetOccupationDuration(
    const ros::Duration& pOccupationDuration)
{
  this->mTargetOccupationInterval.setDuration(pOccupationDuration);
}

template <class Origin, class Target>
void mars::routing::core::Step<Origin, Target>::setPrevious(Step<Target, Origin>* pPrevious)
{
  this->mPrevious = pPrevious;
}

template <class Origin, class Target>
void mars::routing::core::Step<Origin, Target>::setNext(Step<Target, Origin>* pNext)
{
  this->mNext = pNext;
}

template <class Origin, class Target>
void mars::routing::core::Step<Origin, Target>::nullifyPrevious()
{
  this->mPrevious = nullptr;
}

template <class Origin, class Target> void mars::routing::core::Step<Origin, Target>::nullifyNext()
{
  this->mNext = nullptr;
}

template <class Origin, class Target>
mars_routing_msgs::Step mars::routing::core::Step<Origin, Target>::toMsg()
{

  auto tmp3 =
      dynamic_cast<mars::routing::core::TimedStep<mars::routing::common::utility::AffineProfile>*>(
          this);

  mars_routing_msgs::Step lStepMsg;

  const boost::optional<Eigen::Vector3d> lOriginLocation = mOrigin.getLocation();
  const boost::optional<Eigen::Vector3d> lTargetLocation = mTarget.getLocation();

  if (!lOriginLocation || !lTargetLocation)
  {
    // TODO: Handle location request failures!
    return lStepMsg;
  }

  lStepMsg.origin.entity_type.entity_type = mOrigin.getType();
  lStepMsg.origin.id = mOrigin.getId().toMsg();
  lStepMsg.origin_location.point.x = (*lOriginLocation)(0);
  lStepMsg.origin_location.point.y = (*lOriginLocation)(1);
  lStepMsg.origin_location.point.z = (*lOriginLocation)(2);

  lStepMsg.destination.entity_type.entity_type = mTarget.getType();
  lStepMsg.destination.id = mTarget.getId().toMsg();
  lStepMsg.destination_location.point.x = (*lTargetLocation)(0);
  lStepMsg.destination_location.point.y = (*lTargetLocation)(1);
  lStepMsg.destination_location.point.z = (*lTargetLocation)(2);

  lStepMsg.time_interval = mTargetOccupationInterval.toMsg();

  if (auto lAffineProfile = dynamic_cast<
          mars::routing::core::TimedStep<mars::routing::common::utility::AffineProfile>*>(this))
  {
    lStepMsg.time_profile.type = mars_routing_msgs::TimeProfile::PIECEWISE;

    std::vector<double> lLinVelocity, lRotVelocity, lTime;
    double lAccumulatedTime = 0;

    // Reserve 12 entries for 3 ramp profiles (lin-rot-lin with 4 points per ramp)
    lLinVelocity.reserve(12);
    lRotVelocity.reserve(12);
    lTime.reserve(12);

    auto lLinProfile1 = lAffineProfile->getTimeInfo().getEntryProfile();

    lTime.push_back(lAccumulatedTime);
    lLinVelocity.push_back(lLinProfile1.getEntryVelocity());
    lRotVelocity.push_back(0.);

    lTime.push_back(lAccumulatedTime += lLinProfile1.calcAccelerationDuration());
    lLinVelocity.push_back(lLinProfile1.getVelocity());
    lRotVelocity.push_back(0.);

    lTime.push_back(lAccumulatedTime += lLinProfile1.calcMaxVelocityDuration());
    lLinVelocity.push_back(lLinProfile1.getVelocity());
    lRotVelocity.push_back(0.);

    lTime.push_back(lAccumulatedTime += lLinProfile1.calcDecelerationDuration());
    lLinVelocity.push_back(lLinProfile1.getExitVelocity());
    lRotVelocity.push_back(0.);

    // Rotation phase:
    auto lRotProfile = lAffineProfile->getTimeInfo().getRotationProfile();

    lTime.push_back(lAccumulatedTime);
    lLinVelocity.push_back(lLinVelocity.back());
    lRotVelocity.push_back(lRotProfile.getEntryVelocity());

    lTime.push_back(lAccumulatedTime += lRotProfile.calcAccelerationDuration());
    lLinVelocity.push_back(lLinVelocity.back());
    lRotVelocity.push_back(lRotProfile.getVelocity());

    lTime.push_back(lAccumulatedTime += lRotProfile.calcMaxVelocityDuration());
    lLinVelocity.push_back(lLinVelocity.back());
    lRotVelocity.push_back(lRotProfile.getVelocity());

    lTime.push_back(lAccumulatedTime += lRotProfile.calcDecelerationDuration());
    lLinVelocity.push_back(lLinVelocity.back());
    lRotVelocity.push_back(lRotProfile.getExitVelocity());

    // Second linear phase:

    auto lLinProfile2 = lAffineProfile->getTimeInfo().getEntryProfile();

    lTime.push_back(lAccumulatedTime);
    lLinVelocity.push_back(lLinProfile2.getEntryVelocity());
    lRotVelocity.push_back(0.);

    lTime.push_back(lAccumulatedTime += lLinProfile2.calcAccelerationDuration());
    lLinVelocity.push_back(lLinProfile2.getVelocity());
    lRotVelocity.push_back(0.);

    lTime.push_back(lAccumulatedTime += lLinProfile2.calcMaxVelocityDuration());
    lLinVelocity.push_back(lLinProfile2.getVelocity());
    lRotVelocity.push_back(0.);

    lTime.push_back(lAccumulatedTime += lLinProfile2.calcDecelerationDuration());
    lLinVelocity.push_back(lLinProfile2.getExitVelocity());
    lRotVelocity.push_back(0.);

    // std::stringstream ss;
    // ss << "\nt: ";
    // for (auto t : lTime)
    //   ss << t << " ";
    // ss << "\nv: ";
    // for (auto v : lLinVelocity)
    //   ss << v << " ";
    // ss << "\nw: ";
    // for (auto w : lRotVelocity)
    //   ss << w << ", ";
    // MARS_LOG_WARN(ss.str());
    assert(lTime.size() == lLinVelocity.size() && lLinVelocity.size() == lRotVelocity.size());
    lStepMsg.time_profile.time.resize(lTime.size());

    auto lRosTimeIt = lStepMsg.time_profile.time.begin();
    auto lRawIt = lTime.begin();
    while (lRosTimeIt != lStepMsg.time_profile.time.end() && lRawIt != lTime.end())
    {
      *lRosTimeIt = mTargetOccupationInterval.getStartTime() + ros::Duration(*lRawIt);
      ++lRosTimeIt;
      ++lRawIt;
    }

    lStepMsg.time_profile.velocity.resize(lLinVelocity.size());
    auto lTwistIt = lStepMsg.time_profile.velocity.begin();
    auto lLinIt = lLinVelocity.begin();
    auto lRotIt = lRotVelocity.begin();

    while (lTwistIt != lStepMsg.time_profile.velocity.end() && lLinIt != lLinVelocity.end() &&
           lRotIt != lRotVelocity.end())
    {
      lTwistIt->linear.x = *lLinIt;
      lTwistIt->angular.z = *lRotIt;
      ++lTwistIt;
      ++lLinIt;
      ++lRotIt;
    }
  }
  else
  {
    // Step has no valid time info. It might not be derived from mars::routing::core::TimedStep,
    // or it might be, but with a motion profile we did not cast to here.
  }

  return lStepMsg;
}

template <>
boost::optional<Eigen::Vector2d>
mars::routing::core::Step<mars::routing::common::topology::Vertex,
                          mars::routing::common::topology::Edge>::getFootprintIntersection()
{
  const boost::optional<mars::common::geometry::Footprint&> lOriginFootprint =
      mOrigin.getFootprint();
  const boost::optional<Eigen::Vector3d> lOriginLocation = mOrigin.getLocation();
  const boost::optional<Eigen::Vector3d> lTargetLocation = mTarget.getLocation();

  if (!lOriginFootprint || !lOriginLocation || !lTargetLocation)
  {
    return boost::none;
  }

  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lIntersections =
      mars::routing::common::geometry::utilities::getFootprintIntersections(
          *lOriginFootprint, *lOriginLocation, *lTargetLocation);

  if (lIntersections.size() == 1)
  {
    return *lIntersections.begin();
  }

  return boost::none;
}

template <>
boost::optional<Eigen::Vector2d>
mars::routing::core::Step<mars::routing::common::topology::Edge,
                          mars::routing::common::topology::Vertex>::getFootprintIntersection()
{
  const boost::optional<mars::common::geometry::Footprint&> lTargetFootprint =
      mTarget.getFootprint();
  const boost::optional<Eigen::Vector3d> lOriginLocation = mOrigin.getLocation();
  const boost::optional<Eigen::Vector3d> lTargetLocation = mTarget.getLocation();

  if (!lTargetFootprint || !lOriginLocation || !lTargetLocation)
  {
    return boost::none;
  }

  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lIntersections =
      mars::routing::common::geometry::utilities::getFootprintIntersections(
          *lTargetFootprint, *lOriginLocation, *lTargetLocation);

  if (lIntersections.size() == 1)
  {
    Eigen::Vector2d lIntersection = *lIntersections.begin();
    return lIntersection;
  }

  return boost::none;
}

template <>
bool mars::routing::core::Step<mars::routing::common::topology::Vertex,
                               mars::routing::common::topology::Edge>::
    canDeallocateOrigin(const Eigen::Vector2d& pLocation,
                        const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  boost::optional<mars::common::geometry::Footprint&> lOriginFootprint =
      this->mOrigin.getFootprint();

  if (!lOriginFootprint)
  {
    return false;
  }

  boost::geometry::model::polygon<Eigen::Vector2d> lFootprintPolygon;
  boost::geometry::convert(*lOriginFootprint, lFootprintPolygon);
  double lDistanceToFootprintPolygon = boost::geometry::distance(pLocation, lFootprintPolygon);

  if (lDistanceToFootprintPolygon > pRAP.getCollisionRadius())
  {
    return true;
  }

  return false;
}

template <>
bool mars::routing::core::Step<mars::routing::common::topology::Edge,
                               mars::routing::common::topology::Vertex>::
    canDeallocateOrigin(const Eigen::Vector2d& pLocation,
                        const mars::agent::physical::common::RobotAgentProperties& pRAP)
{

  static bool wasWithinFootprint = false;
  bool hasEnoughDistance = false;
  double robotCollisionRadius;

  boost::optional<mars::common::geometry::Footprint&> lFootprint = this->mTarget.getFootprint();

  Eigen::Vector2d footprintIntersection = this->getFootprintIntersection().value();

  if (!lFootprint || lFootprint->size() == 0)
  {
    MARS_LOG_ERROR("Footprint was not available!");
    return false;
  }
  

  // distanceToFootprint = boost::geometry::distance(pLocation, *lFootprint);
  robotCollisionRadius = pRAP.getCollisionRadius();

  // as long as we were not in the entity footprint, check if we are now
  if (!wasWithinFootprint)
  {
    wasWithinFootprint = boost::geometry::within(pLocation, *lFootprint);
  }
  else
  {
    hasEnoughDistance =
        boost::geometry::distance(footprintIntersection, pLocation) > robotCollisionRadius;
  }

  if (wasWithinFootprint && hasEnoughDistance)
  {
    // reset static variable
    wasWithinFootprint = false;
    return true;
  }

  return false;
}

template class mars::routing::core::Step<mars::routing::common::topology::Vertex,
                                         mars::routing::common::topology::Edge>;

template class mars::routing::core::Step<mars::routing::common::topology::Edge,
                                         mars::routing::common::topology::Vertex>;
