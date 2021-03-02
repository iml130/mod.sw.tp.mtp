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

#include "mars_routing_plugin_carp/utilities.h"

boost::optional<ros::Time> mars::routing::plugin::carp::utilities::getReservationStartTime(
    const mars::common::TimeInterval& pTargetFreeTimeInterval, const ros::Time& pTargetEntryTime,
    const ros::Duration& pLeastTargetOccupationDuration)
{
  ros::Duration lLeastReservationDuration = calcLeastReservationDuration(
      pTargetFreeTimeInterval, pTargetEntryTime, pLeastTargetOccupationDuration);

  if ((pTargetEntryTime >= pTargetFreeTimeInterval.getStartTime()) &&
      (pTargetEntryTime + lLeastReservationDuration <= pTargetFreeTimeInterval.getEndTime()))
  {
    return pTargetEntryTime;
  }
  else if ((pTargetEntryTime < pTargetFreeTimeInterval.getStartTime()) &&
           (pTargetFreeTimeInterval.getStartTime() + lLeastReservationDuration <=
            pTargetFreeTimeInterval.getEndTime()))
  {
    return pTargetFreeTimeInterval.getStartTime();
  }

  return boost::none;
}

ros::Duration mars::routing::plugin::carp::utilities::calcLeastReservationDuration(
    const mars::common::TimeInterval& pTargetFreeTimeInterval, const ros::Time& pTargetEntryTime,
    const ros::Duration& pLeastTargetOccupationDuration)
{
  ros::Duration lLeastReservationDuration;
  ros::Time lTime;

  lTime = (pTargetFreeTimeInterval.getStartTime() >= pTargetEntryTime)
              ? pTargetFreeTimeInterval.getStartTime()
              : pTargetEntryTime;

  if (pLeastTargetOccupationDuration == mars::common::TimeInterval::INFINITE_DURATION)
  {
    lLeastReservationDuration = mars::common::TimeInterval::INFINITE_END_TIME - lTime;
  }
  else
  {
    lLeastReservationDuration = pLeastTargetOccupationDuration;
  }

  return lLeastReservationDuration;
}
