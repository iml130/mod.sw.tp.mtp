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

#ifndef MARS_ROUTING_COMMON_UTILITY_MOTIONPROFILE_H
#define MARS_ROUTING_COMMON_UTILITY_MOTIONPROFILE_H

#include <ros/time.h>

namespace mars
{
namespace routing
{
namespace common
{
namespace utility
{
/**
 * @class MotionProfile
 * @brief Abstract class for all possible motion profiles
 */
class MotionProfile
{
public:
  /**
   * @brief getExitVelocity Calculates the exit velocity based on the profile.
   * @return exit velocity. Takes into account, which exit velocity was targeted and if it is
   * achievable in the given timeframe.
   */
  virtual double getExitVelocity() = 0;

  /**
   * @brief getMotionDuration Calculates the duration of the entire motion.
   * @return Duration of the motion. Time unit type is the same as used for the VELOCITY member e.g.
   * for velocity in m/s, the returned type would be seconds.
   */
  virtual ros::Duration getMotionDuration() = 0;

  /**
   * @brief getEntryVelocity Calculates the mean velocity of the vehicle based on the profile.
   * @return mean velocity
   */
  virtual double getMeanVelocity() = 0;

  virtual double getEntryVelocity() = 0;

  double getVelocity() { return VELOCITY; }

protected:
  /**
   * @brief MotionProfile Default constructor for const attributes. Parameter unit types have to
   * match each other!
   * @param motion Angular or linear motion. Angle for angular motion, distance for linear motion.
   * @param velocity Angular or linear velocity. Has to match the motion type (angular/linear).
   */
  MotionProfile(double motion, double velocity) : MOTION(abs(motion)), VELOCITY(abs(velocity))
  {
    // FIXME: set MOTION to 0, if motion is too small?
  }

  double MOTION;
  double VELOCITY;
};
} // namespace utility
} // namespace common
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_COMMON_UTILITY_MOTIONPROFILE_H
