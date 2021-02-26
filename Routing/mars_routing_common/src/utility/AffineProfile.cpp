#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/MotionProfile.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

mars::routing::common::utility::AffineProfile::AffineProfile()
    : mars::routing::common::utility::MotionProfile::MotionProfile(0, 0), ENTRY_VELOCITY(0),
      ACCELERATION(0), DECELERATION(0), EXIT_VELOCITY(0), ACCELERATION_DURATION(0),
      DECELERATION_DURATION(0), ACCELERATION_MOTION(0), DECELERATION_MOTION(0),
      MAX_VELOCITY_MOTION(0), MAX_VELOCITY_DURATION(0)
{
}

mars::routing::common::utility::AffineProfile::AffineProfile(double pMotion, double pEntryVelocity,
                                                             double pTargetVelocity,
                                                             double pExitVelocity,
                                                             double pAcceleration,
                                                             double pDeceleration)
    : mars::routing::common::utility::MotionProfile::MotionProfile(
          pMotion, mars::routing::common::utility::AffineProfile::calcMaxVelocity(
                       pMotion, pEntryVelocity, pTargetVelocity, pExitVelocity, pAcceleration,
                       pDeceleration)),
      ENTRY_VELOCITY(abs(pEntryVelocity)), ACCELERATION(abs(pAcceleration)),
      DECELERATION(abs(pDeceleration)), EXIT_VELOCITY(calcExitVelocity(pExitVelocity)),
      ACCELERATION_DURATION(
          mars::routing::common::utility::AffineProfile::calcAccelerationDuration()),
      DECELERATION_DURATION(
          mars::routing::common::utility::AffineProfile::calcDecelerationDuration()),
      ACCELERATION_MOTION(mars::routing::common::utility::AffineProfile::calcAccelerationMotion()),
      DECELERATION_MOTION(mars::routing::common::utility::AffineProfile::calcDecelerationMotion()),
      MAX_VELOCITY_MOTION(mars::routing::common::utility::AffineProfile::calcMaxVelocityMotion()),
      MAX_VELOCITY_DURATION(
          mars::routing::common::utility::AffineProfile::calcMaxVelocityDuration())
{
}

mars::routing::common::utility::AffineProfile::~AffineProfile() {}

ros::Duration mars::routing::common::utility::AffineProfile::getMotionDuration()
{
  return ACCELERATION_DURATION + MAX_VELOCITY_DURATION + DECELERATION_DURATION;
}

double mars::routing::common::utility::AffineProfile::getMeanVelocity()
{
  return (ENTRY_VELOCITY + EXIT_VELOCITY) / 2.0;
}

double mars::routing::common::utility::AffineProfile::getEntryVelocity() { return ENTRY_VELOCITY; }

double mars::routing::common::utility::AffineProfile::getExitVelocity() { return EXIT_VELOCITY; }
