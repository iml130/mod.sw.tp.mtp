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