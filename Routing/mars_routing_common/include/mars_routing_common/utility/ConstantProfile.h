#ifndef MARS_ROUTING_COMMON_UTILITY_CONSTANTPROFILE_H
#define MARS_ROUTING_COMMON_UTILITY_CONSTANTPROFILE_H

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
 * @class ConstantProfile
 * @brief Motion profile, which has no acceleration and deceleration. Constant velocity at all
 * times.
 */
class ConstantProfile : public MotionProfile
{
public:
  /**
   * @brief ConstantProfile Default constructor initializing all-0 values. Used to predefine empty
   * instances.
   */
  ConstantProfile();

  /**
   * @brief ConstantProfile Constructor for const attributes. Parameter unit types have to match
   * each other!
   * @param motion Angular or linear motion. Angle for angular motion, distance for linear motion.
   * @param entryVelocity Dummy value for interface implementation. Stays ignored.
   * @param targetVelocity Angular or linear velocity used during motion. May be scaled for better
   * estimations.
   * @param exitVelocity Dummy value for interface implementation. Stays ignored.
   * @param acceleration Dummy value for interface implementation. This motion profile has no
   * acceleration.
   * @param deceleration Dummy value for interface implementation. This motion profile has no
   * deceleration.
   */
  ConstantProfile(const double& motion, const double& entryVelocity, const double& targetVelocity,
                  const double& exitVelocity, const double& acceleration,
                  const double& deceleration);
  ~ConstantProfile();

  /**
   * @brief getExitVelocity Calculates the exit velocity based on the profile.
   * @return Constant velocity of the motion profile.
   */
  double getExitVelocity();

  /**
   * @brief getMotionDuration Calculates the duration of the entire motion.
   * @return Duration of the motion. Time unit type is the same as used for the VELOCITY member e.g.
   * for velocity in m/s, the returned type would be seconds.
   */
  ros::Duration getMotionDuration();
};
}  // namespace utility
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_UTILITY_CONSTANTPROFILE_H