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

#include "mars_routing_common/utility/ConstantProfile.h"
#include "mars_routing_common/utility/MotionProfile.h"

mars::routing::common::utility::ConstantProfile::ConstantProfile()
  : mars::routing::common::utility::MotionProfile::MotionProfile(0, 0)
{
}

mars::routing::common::utility::ConstantProfile::ConstantProfile(const double& motion, const double& entryVelocity,
                                                                 const double& targetVelocity,
                                                                 const double& exitVelocity, const double& acceleration,
                                                                 const double& deceleration)
  : mars::routing::common::utility::MotionProfile::MotionProfile(motion, targetVelocity)
{
}

mars::routing::common::utility::ConstantProfile::~ConstantProfile()
{
}

ros::Duration mars::routing::common::utility::ConstantProfile::getMotionDuration()
{
  return ros::Duration(MOTION / VELOCITY);
}

double mars::routing::common::utility::ConstantProfile::getExitVelocity()
{
  return VELOCITY;
}