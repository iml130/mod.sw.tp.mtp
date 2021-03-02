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

#include "mars_common/TimeInterval.h"

#include "mars_routing_core/Route.h"

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"
#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/timeprediction.h"

#include "mars_routing_plugin_carp/Route.h"
#include "mars_routing_plugin_carp/Step.h"
#include "mars_routing_plugin_carp/utilities.h"

#include <ros/time.h>

#include <eigen3/Eigen/Core>

#include "mars_common/Logger.h"

template <class Origin, class Target, class Destination, class MotionProfile>
mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::Route(
    Origin& pOrigin, Destination& pDestination,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, const double pOrientation2D,
    const ros::Time& pStartTime, const ros::Duration& pDestinationReservationDuration)
    : mars::routing::core::Route<Origin, Target, Destination>(pOrigin, pDestination),
      mDestinationReservationDuration(pDestinationReservationDuration)
{
  if (pOrigin == pDestination)
  {
    this->handleEqualDestination();
  }
  else if (pDestination.isTraversable(pRAP))
  {
    this->handleTraversableDestination(pRAP, pOrientation2D, pStartTime);
  }
  else
  {
    this->handleUntraversableDestination();
  }
}

template <class Origin, class Target, class Destination, class MotionProfile>
void mars::routing::plugin::carp::Route<Origin, Target, Destination,
                                        MotionProfile>::handleEqualDestination()
{
  MARS_LOG_WARN("Tried to get a route with identical origin and destination. \n"
                "Origin Id: " +
                this->mOrigin.getId().getUUIDAsString(mars::common::Id::HEXDEC_SPLIT) +
                "\n Destination Id: " +
                this->mDestination.getId().getUUIDAsString(mars::common::Id::HEXDEC_SPLIT));

  this->mTravelInterval = mars::common::TimeInterval(mars::common::TimeInterval::INFINITE_END_TIME,
                                                     mars::common::TimeInterval::INFINITE_DURATION);
  mars::routing::core::Route<Origin, Target, Destination>::mTravelDistance = 0;
}

template <class Origin, class Target, class Destination, class MotionProfile>
void mars::routing::plugin::carp::Route<Origin, Target, Destination,
                                        MotionProfile>::handleUntraversableDestination()
{
  MARS_LOG_WARN("Tried to get a route with untraversable destination. \n"
                "Origin Id: " +
                this->mOrigin.getId().getUUIDAsString(mars::common::Id::HEXDEC_SPLIT) +
                "\n Destination Id: " +
                this->mDestination.getId().getUUIDAsString(mars::common::Id::HEXDEC_SPLIT));

  this->mTravelInterval = mars::common::TimeInterval(mars::common::TimeInterval::INFINITE_END_TIME,
                                                     mars::common::TimeInterval::INFINITE_DURATION);
  mars::routing::core::Route<Origin, Target, Destination>::mTravelDistance = 0;
}

template <class Origin, class Target, class Destination, class MotionProfile>
void mars::routing::plugin::carp::Route<Origin, Target, Destination,
                                        MotionProfile>::handleUnreachableDestination()
{
  MARS_LOG_WARN("Tried to get a route with unreachable destination. \n"
                "Origin Id: " +
                this->mOrigin.getId().getUUIDAsString(mars::common::Id::HEXDEC_SPLIT) +
                "\n Destination Id: " +
                this->mDestination.getId().getUUIDAsString(mars::common::Id::HEXDEC_SPLIT));

  this->mTravelInterval = mars::common::TimeInterval(mars::common::TimeInterval::INFINITE_END_TIME,
                                                     mars::common::TimeInterval::INFINITE_DURATION);
  mars::routing::core::Route<Origin, Target, Destination>::mTravelDistance = 0;
}

template <class Origin, class Target, class Destination, class MotionProfile>
void mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::
    handleTraversableDestination(const mars::agent::physical::common::RobotAgentProperties& pRAP,
                                 const double pOrientation2D, const ros::Time& pStartTime)
{
  MARS_LOG_DEBUG("TRACE");
  std::vector<mars::routing::plugin::carp::PlanningStep*> lOpenList(
      this->initiateRouting(pRAP, pOrientation2D, pStartTime));
  std::vector<mars::routing::plugin::carp::PlanningStep*> lClosedList;

  while (!lOpenList.empty())
  {
    std::sort(lOpenList.begin(), lOpenList.end(),
              [](mars::routing::plugin::carp::PlanningStep* pLeftStep,
                 mars::routing::plugin::carp::PlanningStep* pRightStep) {
                return *pLeftStep < *pRightStep;
              });
    mars::routing::plugin::carp::PlanningStep* lNextStep = lOpenList.back();
    lOpenList.pop_back();
    lClosedList.push_back(lNextStep);

    if (lNextStep->route(pRAP, this->mDestination, lOpenList, lClosedList,
                         this->mDestinationReservationDuration))
    {
      this->initializeSteps(lNextStep);
      printRouteTimeInfo(lNextStep);
      break;
    }
  }

  if (!this->isValid())
  {
    ROS_WARN("Invalid Route, treating as unreachable");
    this->handleUnreachableDestination();
  }

  for (auto itStep : lClosedList)
  {
    delete itStep;
  }

  for (auto itStep : lOpenList)
  {
    delete itStep;
  }

  lOpenList.clear();
  lClosedList.clear();
}

template <class Origin, class Target, class Destination, class MotionProfile>
std::vector<mars::routing::plugin::carp::PlanningStep*>
mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::initiateRouting(
    const mars::agent::physical::common::RobotAgentProperties& pRAP, const double pOrientation2D,
    const ros::Time& pStartTime)
{
  std::vector<mars::routing::plugin::carp::PlanningStep*> lOpenList;
  mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile> lTimeInfo;

  boost::optional<Eigen::Vector3d> lOriginLocation = this->mOrigin.getLocation();
  boost::optional<Eigen::Vector3d> lDestinationLocation = this->mDestination.getLocation();

  if (!lOriginLocation || !lDestinationLocation)
  {
    return lOpenList;
  }

  std::vector<Target> lTraversableTargets = this->mOrigin.getTraversableTargets(pRAP);

  for (Target& iTarget : lTraversableTargets)
  {
    double lTargetEntryVelocity(0);

    boost::optional<ros::Duration> lOriginOccupationDuration =
        mars::routing::common::utility::timeprediction<
            Origin, Target, MotionProfile>::getResourceDuration(this->mOrigin, iTarget,
                                                                pOrientation2D, pRAP,
                                                                lTargetEntryVelocity, lTimeInfo);

    if (!lOriginOccupationDuration)
    {
      continue;
    }

    double lTargetExitVelocity(lTargetEntryVelocity);

    ros::Time lTargetEntryTime = pStartTime + *lOriginOccupationDuration;

    boost::optional<std::vector<mars::common::TimeInterval>> lTargetFreeTimeIntervals =
        iTarget.getFreeTimeIntervals(lTargetEntryTime);

    boost::optional<ros::Duration> lTargetShortestOccupationDuration =
        iTarget.getShortestOccupationDuration(this->mOrigin, pRAP, lTargetExitVelocity, lTimeInfo);

    boost::optional<Eigen::Vector3d> lTargetLocation = iTarget.getLocation();

    if (!lTargetFreeTimeIntervals || !lTargetShortestOccupationDuration || !lTargetLocation)
    {
      continue;
    }

    for (const auto& iTargetFreeTimeInterval : *lTargetFreeTimeIntervals)
    {
      boost::optional<ros::Time> lTargetReservationStartTime =
          mars::routing::plugin::carp::utilities::getReservationStartTime(
              iTargetFreeTimeInterval, lTargetEntryTime, *lTargetShortestOccupationDuration);

      if (!lTargetReservationStartTime)
      {
        continue;
      }

      mars::common::TimeInterval lTargetReservationInterval(*lTargetReservationStartTime,
                                                            *lTargetShortestOccupationDuration);

      ros::Duration lOriginMotionDuration =
          *lOriginOccupationDuration + *lTargetShortestOccupationDuration +
          (*lTargetReservationStartTime - lTargetEntryTime); // g costs

      double lDestinationDistance = (*lTargetLocation - *lDestinationLocation).norm(); // h costs
      ros::Duration lDestinationMotionDuration = mars::routing::common::utility::timeprediction<
          Target, Origin, MotionProfile>::getMotionDuration(lDestinationDistance, pRAP);

      mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>* lOpenPlanningStep =
          new mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>(
              this->mOrigin, iTarget, lTargetReservationInterval, iTargetFreeTimeInterval,
              lTargetEntryVelocity, lTargetExitVelocity, lOriginMotionDuration,
              lDestinationMotionDuration);

      lOpenPlanningStep->setTimeInfo(lTimeInfo);

      lOpenList.push_back(lOpenPlanningStep);
    }
  }

  return lOpenList;
}

template <class Origin, class Target, class Destination, class MotionProfile>
void mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::
    initializeSteps(mars::routing::plugin::carp::PlanningStep* pLastStep)
{

  // Deep copy the the steps en route to the destination, so that the doubly
  // linked list making up the route consists of steps seperate from the steps
  // employed in A*, making it safe to delete the latter

  this->mStepCount++;
  mars::routing::plugin::carp::PlanningStep* lPreviousStep = pLastStep->getPlannedPrevious();

  mars::routing::plugin::carp::PlanningStep *lPersistentLastStep, *lPersistentPreviousStep;

  lPersistentLastStep = pLastStep->clone();

  while (lPreviousStep != nullptr)
  {
    this->mStepCount++;

    lPersistentPreviousStep = lPreviousStep->clone();

    if (!checkStepValid(*pLastStep))
    {
      this->mValid = false;
      return;
    }

    // printStepTimeInfo(pLastStep);

    lPersistentLastStep->setPlannedPrevious(lPersistentPreviousStep);
    lPersistentPreviousStep->setPlannedNext(lPersistentLastStep);

    pLastStep = lPreviousStep;
    lPersistentLastStep = lPersistentPreviousStep;

    lPreviousStep = pLastStep->getPlannedPrevious();
  }

  this->mFirstStep =
      (mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>*)lPersistentPreviousStep;

  this->mValid = true;
}

template <class Origin, class Target, class Destination, class MotionProfile>
void mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::
    printRouteTimeInfo(mars::routing::plugin::carp::PlanningStep* pLastStep)
{
  std::vector<mars::routing::plugin::carp::PlanningStep*> lRoute;
  mars::routing::plugin::carp::PlanningStep* lStep = pLastStep;
  do
  {
    lRoute.push_back(lStep);
    lStep = lStep->getPlannedPrevious();
  } while (lStep != nullptr);

  for (std::vector<mars::routing::plugin::carp::PlanningStep*>::reverse_iterator it =
           lRoute.rbegin();
       it != lRoute.rend(); it++)
  {
    printStepTimeInfo(*it);
  }
}

template <class Origin, class Target, class Destination, class MotionProfile>
bool mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::reserve(
    const mars::common::Id& pAgentId)
{
  for (mars::routing::core::IterationStep& iStep : *this)
  {
    MARS_LOG_DEBUG("Adding reservation at "
                   << iStep.getTargetOccupationInterval().getStartTime() << " for "
                   << iStep.getTargetOccupationInterval().getDuration() << " on Node with ID "
                   << iStep.getTarget().getId().getUUIDAsString(
                          mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                   << " and Type " << iStep.getTarget().getType());

    if (!iStep.getTarget().addReservation(pAgentId, this->mId, iStep.getTargetOccupationInterval()))
    {
      MARS_LOG_WARN("Could not add reservation, routing failed, clear previous "
                    "reservations");

      for (mars::routing::core::IterationStep& jStep : *this)
      {
        if (!jStep.getTarget().deleteReservation(pAgentId, this->mId))
        {
          return false;
        }
      }
      return false;
    }
  }

  return true;
}

template <class Origin, class Target, class Destination, class MotionProfile>
bool mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::checkStepValid(
    const mars::routing::plugin::carp::PlanningStep& pStep)
{
  bool lStepIsValid = false;
  mars::routing::plugin::carp::PlanningStep* lPreviousStep = pStep.getPlannedPrevious();

  if (pStep.getTargetFreeInterval().getStartTime() <=
          pStep.getPlannedTargetOccupationInterval().getStartTime() &&
      pStep.getPlannedTargetOccupationInterval().getEndTime() <=
          pStep.getTargetFreeInterval().getEndTime())
  {
    lStepIsValid = true;

    if (lPreviousStep != nullptr &&
        !(pStep.getPlannedTargetOccupationInterval().getStartTime() <
          lPreviousStep->getPlannedTargetOccupationInterval().getEndTime()))
    {
      lStepIsValid = false;
      MARS_LOG_ERROR("Current occupation interval starts after previous occupation interval ends");
    }
  }
  else
  {
    if (pStep.getTargetFreeInterval().getStartTime() >
            pStep.getPlannedTargetOccupationInterval().getStartTime() &&
        pStep.getPlannedTargetOccupationInterval().getEndTime() >
            pStep.getTargetFreeInterval().getEndTime())
    {
      MARS_LOG_ERROR("Occupation interval is bigger than freetime interval (both directions)");
    }
    else if (pStep.getTargetFreeInterval().getStartTime() >
             pStep.getPlannedTargetOccupationInterval().getStartTime())
    {
      MARS_LOG_ERROR("Occupation interval start is earlier than freetime interval start");
    }
    else
    {
      MARS_LOG_ERROR("Occupation interval end is later than freetime interval end");
    }
  }

  return lStepIsValid;
}

template <class Origin, class Target, class Destination, class MotionProfile>
void mars::routing::plugin::carp::Route<Origin, Target, Destination, MotionProfile>::
    printStepTimeInfo(mars::routing::plugin::carp::PlanningStep* pStep)
{
  mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>* tmp =
      dynamic_cast<mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>*>(pStep);

  mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>* tmp2 =
      dynamic_cast<mars::routing::plugin::carp::Step<Target, Origin, MotionProfile>*>(pStep);

  mars::routing::common::utility::TimeInfo<MotionProfile> lLastStepTimeInfo;

  if (tmp != nullptr)
  {
    lLastStepTimeInfo = tmp->getTimeInfo();
    ROS_DEBUG_STREAM("Reservation " << tmp->getTargetOccupationInterval().getStartTime() << " for "
                                    << tmp->getTargetOccupationInterval().getDuration()
                                    << " on Node with ID "
                                    << tmp->getTarget().getId().getUUIDAsString(
                                           mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                                    << " and Type " << tmp->getTarget().getType());
  }
  else if (tmp2 != nullptr)
  {
    lLastStepTimeInfo = tmp2->getTimeInfo();
    ROS_DEBUG_STREAM("Reservation " << tmp2->getTargetOccupationInterval().getStartTime() << " for "
                                    << tmp2->getTargetOccupationInterval().getDuration()
                                    << " on Node with ID "
                                    << tmp2->getTarget().getId().getUUIDAsString(
                                           mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                                    << " and Type " << tmp2->getTarget().getType());
  }
  else
  {
    ROS_WARN_STREAM("STEP PRINT SKIPPED");
    return;
  }

  ROS_DEBUG_STREAM(
      "TimeInfo:\n"
      << "StepOrigin: " << pStep->getOriginId().getUUIDAsString(mars::common::Id::HEXDEC) << "\n"
      << "StepTarget: " << pStep->getTargetId().getUUIDAsString(mars::common::Id::HEXDEC) << "\n"
      << "EntryMotion: " << lLastStepTimeInfo.getEntryMotion() << "m \n"
      << "EntryProfile EntryVelocity: " << lLastStepTimeInfo.getEntryProfile().getEntryVelocity()
      << "m/s \n"
      << "EntryProfile ExitVelocity: " << lLastStepTimeInfo.getEntryProfile().getExitVelocity()
      << "m/s \n"
      << "AngularMotion: " << lLastStepTimeInfo.getAngularMotion() << "rad \n"
      << "RotationProfile EntryVelocity: "
      << lLastStepTimeInfo.getRotationProfile().getEntryVelocity() << "m/s \n"
      << "RotationProfile ExitVelocity: "
      << lLastStepTimeInfo.getRotationProfile().getExitVelocity() << "m/s \n"
      << "ExitMotion: " << lLastStepTimeInfo.getExitMotion() << "m \n"
      << "ExitProfile EntryVelocity: " << lLastStepTimeInfo.getExitProfile().getEntryVelocity()
      << "m/s \n"
      << "ExitProfile ExitVelocity: " << lLastStepTimeInfo.getExitProfile().getExitVelocity()
      << "m/s \n");
}

template class mars::routing::plugin::carp::Route<
    mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
    mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>;

template class mars::routing::plugin::carp::Route<
    mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
    mars::routing::common::topology::Edge, mars::routing::common::utility::AffineProfile>;

template class mars::routing::plugin::carp::Route<
    mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
    mars::routing::common::topology::Vertex, mars::routing::common::utility::AffineProfile>;

template class mars::routing::plugin::carp::Route<
    mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
    mars::routing::common::topology::Edge, mars::routing::common::utility::AffineProfile>;
