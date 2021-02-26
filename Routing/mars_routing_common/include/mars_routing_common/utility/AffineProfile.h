#ifndef MARS_ROUTING_COMMON_UTILITY_AFFINEPROFILE_H
#define MARS_ROUTING_COMMON_UTILITY_AFFINEPROFILE_H

#include "mars_routing_common/utility/MotionProfile.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace utility
{
/**
 * @class AffineProfile
 * @brief Motion profile, which has an affine acceleration over time.
 */
class AffineProfile : public MotionProfile
{
public:
  /**
   * @brief AffineProfile Default constructor initializing all-0 values. Used to predefine empty
   * instances.
   */
  AffineProfile();

  /**
   * @brief AffineProfile Constructor for const attributes. Parameter unit types have to match each
   * other!
   * @param pMotion Angular or linear motion. Angle for angular motion, distance for linear motion.
   * @param pEntryVelocity Velocity at the beginning of the motion.
   * @param pTargetVelocity Targeted velocity to reach during motion. May not get reached in time,
   * if entry velocity and motion are too low.
   * @param pExitVelocity Targeted velocity to end the motion with. May not get reached in time, if
   * entry velocity and motion are too low.
   * @param pAcceleration Highest possible acceleration to reach while accelerating. Has to be
   * positive.
   * @param pDeceleration Highest possible deceleration to reach while braking. Has to be positive.
   */
  AffineProfile(double pMotion, double pEntryVelocity, double pTargetVelocity, double pExitVelocity,
                double pAcceleration, double pDeceleration);
  ~AffineProfile();

  /**
   * @brief getExitVelocity Calculates the exit velocity based on the profile.
   * @return True exit velocity. Takes into account, which exit velocity was targeted and if it is
   * achievable in the given timeframe based on the motion.
   */
  double getExitVelocity();

  /**
   * @brief getMotionDuration Calculates the duration of the entire motion.
   * @return Duration of the motion. Time unit type is the same as used for the VELOCITY member e.g.
   * for velocity in m/s, the returned type would be seconds.
   */
  ros::Duration getMotionDuration();

  /**
   * @brief getMeanVelocity Calculates the mean velocity of the vehicle.
   * @return mean velocity in m/s
   */
  double getMeanVelocity();

  double getEntryVelocity();

private:
  double ENTRY_VELOCITY;
  double ACCELERATION;
  double DECELERATION;
  double EXIT_VELOCITY;

  ros::Duration ACCELERATION_DURATION;
  ros::Duration DECELERATION_DURATION;

  double ACCELERATION_MOTION;
  double DECELERATION_MOTION;
  double MAX_VELOCITY_MOTION;

  ros::Duration MAX_VELOCITY_DURATION;

public:
  /**
   * @brief Helper function. Calculates the peaked velocity during the motion.
   * @param pMotion Angular or linear motion. Angle for angular motion, distance for linear motion.
   * @param pEntryVelocity Initial velocity of the profile.
   * @param pTargetVelocity Targeted velocity to reach during motion. May not get reached in time,
   * if entry velocity and motion are too low.
   * @param pExitVelocity Targeted velocity to end the motion with. May not get reached in time, if
   * entry velocity and motion are too low.
   * @param pAcceleration Constant acceleration during the profiled motion.
   * @param pDeceleration Constant deceleration during the profiled motion.
   * @return Highest reached velocity during the motion. Equals the targeted velocity, if the motion
   * lasts long enough to full accelerate from the entry velocity to the targeted velocity and after
   * that fully decelerate to the exit velocity.
   */
  inline double calcMaxVelocity(double pMotion, double pEntryVelocity, double pTargetVelocity,
                                double pExitVelocity, double pAcceleration, double pDeceleration)
  {
    double acceleration_sum = pAcceleration + pDeceleration;
    if (acceleration_sum == 0 || pMotion == 0)
    {
      return pEntryVelocity;
    }
    return std::min(std::min(pTargetVelocity, sqrt((2 * pAcceleration * pDeceleration * pMotion +
                                                    pAcceleration * pow(pExitVelocity, 2) +
                                                    pDeceleration * pow(pEntryVelocity, 2)) /
                                                   acceleration_sum)),
                    sqrt(2 * pMotion * pAcceleration + pEntryVelocity));
  }

  /**
   * @brief Helper function to calculate the true exit velocity from a given targeted exit velocity.
   * @param pExitvelocity The targeted exit velocity.
   * @return The velocity at the end of the profile.
   */
  inline double calcExitVelocity(double pExitVelocity) { return std::min(VELOCITY, pExitVelocity); }

  /**
   * @brief Helper function to calculate the acceleration duration.
   * @return The time it takes to accelerate from ENTRY_VELOCITY to VELOCITY.
   */
  inline double calcAccelerationDuration()
  {
    if (ACCELERATION == 0)
    {
      return 0;
    }
    return (VELOCITY - ENTRY_VELOCITY) / ACCELERATION;
  }

  /**
   * @brief Helper function to calculate the deceleration duration.
   * @return The time it takes to decelerate from VELOCITY to EXIT_VELOCITY.
   */
  inline double calcDecelerationDuration()
  {
    if (DECELERATION == 0)
    {
      return 0;
    }
    return ((VELOCITY - EXIT_VELOCITY) / DECELERATION);
  }

  /**
   * @brief Helper function to calculate the maximum velocity duration.
   * @return The time the motion goes on with the targeted velocity. 0, if the targeted velocity
   * doesn't get reached during the motion.
   */
  inline double calcMaxVelocityDuration()
  {
    if (VELOCITY == 0)
    {
      return 0;
    }
    return MAX_VELOCITY_MOTION / VELOCITY;
  }

  /**
   * @brief Helper function to calculate the acceleration motion.
   * @return Angular or linear motion, depending on the provided motion, during acceleration.
   */
  inline double calcAccelerationMotion()
  {
    return ACCELERATION_DURATION.toSec() * (VELOCITY + ENTRY_VELOCITY) / 2;
  }

  /**
   * @brief Helper function to calculate the deceleration motion.
   * @return Angular or linear motion, depending on the provided motion, during deceleration.
   */
  inline double calcDecelerationMotion()
  {
    return DECELERATION_DURATION.toSec() * (VELOCITY + EXIT_VELOCITY) / 2;
  }

  /**
   * @brief Helper function to calculate the maximum velocity motion.
   * @return Angular or linear motion, depending on the provided motion, while moving with the
   * maximum velocity.
   */
  inline double calcMaxVelocityMotion()
  {
    return MOTION - ACCELERATION_MOTION - DECELERATION_MOTION;
  }
};
} // namespace utility
} // namespace common
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_COMMON_UTILITY_AFFINEPROFILE_H
