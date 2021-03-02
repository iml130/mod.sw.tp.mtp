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

#include "mars_routing_plugin_carp/Step.h"
#include "mars_routing_plugin_carp/utilities.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/timeprediction.h"

#include "mars_common/Logger.h"

template <class Origin, class Target, class MotionProfile>
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::Step(
    Origin& pOrigin, Target& pTarget, const mars::common::TimeInterval& pTargetOccupationInterval,
    const mars::common::TimeInterval& pTargetFreeInterval, const double& pEntryVelocity,
    const double& pExitVelocity, const ros::Duration& pOriginMotionDuration,
    const ros::Duration& pDestinationMotionDuration,
    mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>* pPrevious,
    mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>* pNext)
    : mars::routing::core::Step<Origin, Target>::Step(pOrigin, pTarget, pTargetOccupationInterval,
                                                      pPrevious, pNext),
      mTargetFreeInterval(pTargetFreeInterval), mEntryVelocity(pEntryVelocity),
      mExitVelocity(pExitVelocity), mOriginMotionDuration(pOriginMotionDuration),
      mDestinationMotionDuration(pDestinationMotionDuration)
{
}

template <class Origin, class Target, class MotionProfile>
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>*
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::clone() const
{
  Origin lOrigin = this->mOrigin;
  Target lTarget = this->mTarget;

  mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>* lClonedStep =
      new mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>(
          lOrigin, lTarget, this->mTargetOccupationInterval, this->mTargetFreeInterval,
          this->mEntryVelocity, this->mExitVelocity, this->mOriginMotionDuration,
          this->mDestinationMotionDuration);

  lClonedStep->mPrevious = this->mPrevious;

  lClonedStep->mTimeInfo = this->mTimeInfo;

  return lClonedStep;
}

template <class Origin, class Target, class MotionProfile>
mars::common::Id
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getOriginId() const
{
  return this->mOrigin.getId();
}

template <class Origin, class Target, class MotionProfile>
mars::common::Id
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getTargetId() const
{
  return this->mTarget.getId();
}

template <class Origin, class Target, class MotionProfile>
const mars::common::TimeInterval&
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getTargetFreeInterval() const
{
  return this->mTargetFreeInterval;
}

template <class Origin, class Target, class MotionProfile>
const double&
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getEntryVelocity() const
{
  return this->mExitVelocity;
}

template <class Origin, class Target, class MotionProfile>
const double&
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getExitVelocity() const
{
  return this->mExitVelocity;
}

template <class Origin, class Target, class MotionProfile>
const ros::Duration&
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getOriginMotionDuration() const
{
  return this->mOriginMotionDuration;
}

template <class Origin, class Target, class MotionProfile>
const ros::Duration&
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getDestinationMotionDuration()
    const
{
  return this->mDestinationMotionDuration;
}

template <class Origin, class Target, class MotionProfile>
bool mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::
operator<(const mars::routing::plugin::carp::PlanningStep& pOtherStep) const
{
  return this->getOriginMotionDuration() + this->getDestinationMotionDuration() >
         pOtherStep.getOriginMotionDuration() + pOtherStep.getDestinationMotionDuration();
}

template <class Origin, class Target, class MotionProfile>
bool mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::route(
    const mars::agent::physical::common::RobotAgentProperties& pRAP,
    mars::routing::common::topology::Entity& pDestination,
    std::vector<mars::routing::plugin::carp::PlanningStep*>& pOpenList,
    std::vector<mars::routing::plugin::carp::PlanningStep*>& pClosedList,
    const ros::Duration& pDestinationReservationDuration)
{
  MARS_LOG_DEBUG("Planning Routing Step to " << this->mTarget.getId().getUUIDAsString(
                     mars::common::Id::UUIDFormat::HEXDEC_SPLIT));

  if (this->mTarget == pDestination)
  {
    this->setTargetOccupationDuration(pDestinationReservationDuration);
    return true;
  }

  mars::common::Id lTargetId = this->mTarget.getId();

  boost::optional<Eigen::Vector3d> lTargetLocation = this->mTarget.getLocation();
  boost::optional<Eigen::Vector3d> lDestinationLocation = pDestination.getLocation();

  if (!lTargetLocation || !lDestinationLocation)
  {
    return false;
  }

  auto lPotentialTargets = this->mTarget.getTraversableTargets(pRAP);

  for (auto& iNextTarget : lPotentialTargets)
  {
    if (iNextTarget == this->mOrigin)
    {
      continue;
    }

    double lExitVelocity = this->mEntryVelocity;

    boost::optional<ros::Duration> lTargetOccupationDuration = this->mTarget.getOccupationDuration(
        this->mOrigin, iNextTarget, pRAP, lExitVelocity, this->mTimeInfo);

    // Search for the needed dimensions of the vehicle
    double lRobotFrontLength = pRAP.getFrontLength();

    double lRobotBackLength = pRAP.getBackLength();

    // Add the time on top of the resveration
    ros::Duration lBackOfVehicleDuration =
        ros::Duration(std::abs(lRobotFrontLength) / lExitVelocity);

    ros::Duration lFrontVehicleDuration =
        ros::Duration(std::abs(lRobotBackLength) / this->mEntryVelocity);

    ros::Time lTargetEntryTime = this->mTargetOccupationInterval.getStartTime();

    if (!lTargetOccupationDuration)
    {
      ROS_DEBUG_STREAM(
          "Dropped Step, no lTargetOccupationDuration \n"
          "Von: "
          << this->mTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
          << "\n"
          << "Nach: "
          << iNextTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
          << "\n");
      continue;
    }

    ros::Time lTargetExitTime = lTargetEntryTime + *lTargetOccupationDuration;

    if (lTargetExitTime > this->mTargetFreeInterval.getEndTime() ||
        lTargetEntryTime < this->mTargetFreeInterval.getStartTime())
    {
      ROS_DEBUG_STREAM(
          "Dropped Step, nicht iNextTargetim free time interval \n"
          "Von: "
          << this->mTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
          << "\n"
          << "Nach: "
          << iNextTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) << "\n"
          << "targetExitTime: " << lTargetExitTime << "\n"
          << "targetEntryTime: " << lTargetEntryTime << "\n"
          << "free time interval von .. bis: " << this->mTargetFreeInterval.getStartTime() << "\n"
          << this->mTargetFreeInterval.getEndTime());
      continue;
    }

    //correction of Motion costs of the whole route
    // substract the shortestOccupation duration
    this->mOriginMotionDuration -= this->getTargetOccupationInterval().getDuration();
    //and add the actual occupation duration as costs
    this->mOriginMotionDuration += *lTargetOccupationDuration;

    this->mExitVelocity = lExitVelocity;
    this->setTargetOccupationDuration(*lTargetOccupationDuration);
    this->mTargetOccupationInterval.setStartTime(lTargetEntryTime);

    double lNextTargetEntryVelocity = lExitVelocity;
    double lNextTargetExitVelocity = lNextTargetEntryVelocity;

    boost::optional<std::vector<mars::common::TimeInterval>> lNextTargetFreeTimeIntervals =
        iNextTarget.getFreeTimeIntervals(lTargetExitTime -
                                         (lBackOfVehicleDuration + lFrontVehicleDuration));

     boost::optional<ros::Duration> lNextTargetShortestOccupationDuration;

    if(iNextTarget == pDestination)
    {
      // our occupation duration is given to us from the route request for the destination
      lNextTargetShortestOccupationDuration = pDestinationReservationDuration;
    }
    else
    {
       mars::routing::common::utility::TimeInfo<MotionProfile> lEdgeTimeInfo;
    lNextTargetShortestOccupationDuration =
        iNextTarget.getShortestOccupationDuration(this->mTarget, pRAP, lNextTargetExitVelocity,
                                                  lEdgeTimeInfo);
    }

    boost::optional<Eigen::Vector3d> lNextTargetLocation = iNextTarget.getLocation();

    if (!lNextTargetShortestOccupationDuration || !lNextTargetLocation ||
        !lNextTargetFreeTimeIntervals)
    {
      ROS_DEBUG_STREAM(
          "Dropped Step, irgendwas hat gefehlt \n"
          "Von: "
          << this->mTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
          << "\n"
          << "Nach: "
          << iNextTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
          << "\n");
      continue;
    }

    for (const auto& iNextTargetFreeTimeInterval : *lNextTargetFreeTimeIntervals)
    {
      boost::optional<ros::Time> lNextTargetReservationStartTime =
          mars::routing::plugin::carp::utilities::getReservationStartTime(
              iNextTargetFreeTimeInterval,
              lTargetExitTime - (lBackOfVehicleDuration + lFrontVehicleDuration),
              *lNextTargetShortestOccupationDuration);        

      if (!lNextTargetReservationStartTime ||
          *lNextTargetReservationStartTime >= this->mTargetFreeInterval.getEndTime())
      {
        ROS_DEBUG_STREAM(
            "Dropped Step, next start time außerhalb free time interval\n"
            "Von: "
            << this->mTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
            << "\n"
            << "Nach: "
            << iNextTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
            << "\n");
        continue;
      }

      ros::Duration WaitPeriod = *lNextTargetReservationStartTime - lTargetExitTime;

      if (*lNextTargetReservationStartTime >= this->mTargetOccupationInterval.getEndTime())
      {
        // stretch target reservation to connect to next start timestamp
        if ((*lNextTargetReservationStartTime + lBackOfVehicleDuration + lFrontVehicleDuration) <=
            this->mTargetFreeInterval.getEndTime())
        {
          ros::Duration diffDuration =
              *lNextTargetReservationStartTime - this->mTargetOccupationInterval.getEndTime();

          // current duration + diff between current end and new start + overlap (one vehicle length
          ros::Duration fittedTargetDuration = this->mTargetOccupationInterval.getDuration() +
                                               diffDuration +
                                               (lBackOfVehicleDuration + lFrontVehicleDuration);
          this->setTargetOccupationDuration(fittedTargetDuration);
        }
        else
        {
          continue;
        }
      }

      mars::common::Id lNextTargetId = iNextTarget.getId();

      if (iNextTarget == pDestination &&
          iNextTargetFreeTimeInterval.getEndTime() <
              *lNextTargetReservationStartTime + pDestinationReservationDuration)
      {
        ROS_DEBUG_STREAM(
            "Dropped Step \n"
            "Von: "
            << this->mTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
            << "\n"
            << "Nach: "
            << iNextTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
            << "\n");

        continue;
      }

      auto lOpenListFetch =
          std::find_if(pOpenList.begin(), pOpenList.end(),
                       [&iNextTargetFreeTimeInterval, &lTargetId, &lNextTargetId](
                           const mars::routing::plugin::carp::PlanningStep* pOpenStep) {
                         return pOpenStep->getTargetFreeInterval() == iNextTargetFreeTimeInterval &&
                                pOpenStep->getOriginId() == lTargetId &&
                                pOpenStep->getTargetId() == lNextTargetId;
                       });

      auto lClosedListFetch = std::find_if(
          pClosedList.begin(), pClosedList.end(),
          [&iNextTargetFreeTimeInterval, &lTargetId,
           &lNextTargetId](const mars::routing::plugin::carp::PlanningStep* pClosedStep) {
            return pClosedStep->getTargetFreeInterval() == iNextTargetFreeTimeInterval &&
                   pClosedStep->getOriginId() == lTargetId &&
                   pClosedStep->getTargetId() == lNextTargetId;
          });

      if (lOpenListFetch != pOpenList.end() || lClosedListFetch != pClosedList.end())
      {
        continue;
      }

      mars::common::TimeInterval lNextTargetReservationInterval(
          *lNextTargetReservationStartTime, *lNextTargetShortestOccupationDuration);

      ros::Duration lOriginMotionDuration;

      if(lNextTargetId == pDestination.getId())
      {
        lOriginMotionDuration = this->mOriginMotionDuration; // g costs
      }
      else
      {
          lOriginMotionDuration =
          this->mOriginMotionDuration + *lNextTargetShortestOccupationDuration + WaitPeriod; // g costs
      }

      double lDestinationDistance =
          (*lNextTargetLocation - *lDestinationLocation).norm(); // h costs
      ros::Duration lDestinationMotionDuration = mars::routing::common::utility::timeprediction<
          Target, Origin, MotionProfile>::getMotionDuration(lDestinationDistance, pRAP);

      auto lClonedStep = this->clone(); // Pass a copy of *this to the next step, as *this will change depending on the next step
      pClosedList.push_back(lClonedStep); // Copy goes directly to closed list: Its next step is already defined and *this is already on open list
      mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>* lOpenPlanningStep =
          new mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>(
              this->mTarget, iNextTarget, lNextTargetReservationInterval,
              iNextTargetFreeTimeInterval, lNextTargetEntryVelocity, lNextTargetExitVelocity,
              lOriginMotionDuration, lDestinationMotionDuration, lClonedStep);

      ROS_DEBUG_STREAM(
          "Neuer Step \n"
          "Von: "
          << this->mTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
          << "\n"
          << "Nach: "
          << iNextTarget.getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
          << "\n");

      pOpenList.push_back(lOpenPlanningStep);
    }
  }

  return false;
}

template <class Origin, class Target, class MotionProfile>
const mars::common::TimeInterval&
mars::routing::plugin::carp::Step<Origin, Target,
                                  MotionProfile>::getPlannedTargetOccupationInterval() const
{
  return mars::routing::core::Step<Origin, Target>::getTargetOccupationInterval();
}

template <class Origin, class Target, class MotionProfile>
mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>*
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getPlannedPrevious() const
{
  return (mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>*)
      mars::routing::core::Step<Origin, Target>::getPrevious();
}

template <class Origin, class Target, class MotionProfile>
mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>*
mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getPlannedNext() const
{
  return (mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>*)
      mars::routing::core::Step<Origin, Target>::getNext();
}

template <class Origin, class Target, class MotionProfile>
void mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::setPlannedPrevious(
    const mars::routing::plugin::carp::PlanningStep* pPrevious)
{
  mars::routing::core::Step<Origin, Target>::setPrevious(
      (mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>*)pPrevious);
}

template <class Origin, class Target, class MotionProfile>
void mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::setPlannedNext(
    const mars::routing::plugin::carp::PlanningStep* pNext)
{
  mars::routing::core::Step<Origin, Target>::setNext(
      (mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>*)pNext);
}

template <class Origin, class Target, class MotionProfile>
void mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::
    setPlannedTargetOccupationDuration(const ros::Duration& pTargetOccupationDuration)
{
  mars::routing::core::Step<Origin, Target>::setTargetOccupationDuration(pTargetOccupationDuration);
}

// template <class Origin, class Target, class MotionProfile>
// mars::routing::common::utility::TimeInfo<MotionProfile>
// mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::getTimeInfo() const
// {
//   return this->mTimeInfo;
// }

// template <class Origin, class Target, class MotionProfile>
// void mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>::setTimeInfo(
//     const mars::routing::common::utility::TimeInfo<MotionProfile>& pTimeInfo)
// {
//   this->mTimeInfo = pTimeInfo;
// }

template class mars::routing::plugin::carp::Step<mars::routing::common::topology::Vertex,
                                                 mars::routing::common::topology::Edge,
                                                 mars::routing::common::utility::AffineProfile>;

template class mars::routing::plugin::carp::Step<mars::routing::common::topology::Edge,
                                                 mars::routing::common::topology::Vertex,
                                                 mars::routing::common::utility::AffineProfile>;
