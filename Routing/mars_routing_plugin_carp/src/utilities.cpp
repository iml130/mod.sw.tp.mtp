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
