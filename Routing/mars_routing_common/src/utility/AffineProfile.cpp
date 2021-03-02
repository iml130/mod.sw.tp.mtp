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
