#ifndef MARS_ROUTING_COMMON_UTILITY_TIMEPREDICTION_H
#define MARS_ROUTING_COMMON_UTILITY_TIMEPREDICTION_H

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_common/topology/Vertex.h"
#include <mars_routing_common/utility/TimeInfo.h>

#include <ros/time.h>

#include <boost/optional.hpp>

namespace mars
{
namespace routing
{
namespace common
{
namespace utility
{
/**
 * @brief Partially private namespace, implementing time request functions for routing calculation.
 */
template <class Origin, class Target, class Profile>
class timeprediction
{
public:
  /**
   * @brief getResourceDuration Calculates the total estimated time an agent would occupy a vertex
   * given a previous vertex on a routing path.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   * @param pPreviousEntity Previous topology entity on the routing path.
   * @param pRAP Estimated robot motion parameters. Required for accurate time estimation. Can be
   * scaled or biased for more accuracy.
   * @param pVelocity Entry velocity into the resource. Calculated in previous iterations. Turned
   * into exit velocity after calculation!
   * @return Total estimated time an agent would occupy the resource while following a path. The
   * time unit depends on the units used in pRAP. Those units have to be consistent and matching!
   */
  static boost::optional<ros::Duration>
  getResourceDuration(Origin& pCurrentEntity,
                      Target& pNextEntity,
                      Target& pPreviousEntity,
                      const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity, mars::routing::common::utility::TimeInfo<Profile>& pTimeInfo);

  /**
   * @brief getResourceDuration Calculates the total estimated time an agent would occupy an initial
   * vertex given the current orientation of the agent.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   * @param pOrientation Current orientation of the agent on the current vertex.
   * @param pRAP Estimated robot motion parameters. Required for accurate time estimation. Can be
   * scaled or biased for more accuracy.
   * @param pVelocity Reference to velocity variable, which will contain the exit velocity
   * out of the resource after this calculation.
   * @return Total estimated time an agent would occupy the resource while following a path starting
   * on that resource. The time unit depends on the units used in pRAP. Those units have to be
   * consistent and matching!
   */
  static boost::optional<ros::Duration> getResourceDuration(
      Origin& pCurrentEntity,
      Target& pNextEntity, double pOrientation,
      const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity, mars::routing::common::utility::TimeInfo<Profile>& pTimeInfo);

  /**
   * @brief TODO: doxy
   *
   */
  static ros::Duration getMotionDuration(double pMotion,
                                         const mars::agent::physical::common::RobotAgentProperties& pRAP);

private:

  /**
   * @brief timeprediction Private constructor to ensure the class stays static and is merely a
   * namespace with some private functions.
   */
  timeprediction();


  /**
   * @brief setMotion Calculates angular and linear motions inside a vertex.
   * @param pAngularMotion Angular motion to calculate. Sets reference inside this function.
   * @param pEntryMotion Linear entry motion to calculate. Sets reference inside this function.
   * @param pExitMotion Linear exit motion to calculate. Sets reference inside this function.
   * @param pRotationAngle Required angle to rotate on the resource.
   * @param pRAP Estimated robot motion parameters. Required for accurate time estimation. Can be
   * scaled or biased for more accuracy.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   * @param pPreviousEntity Previous topology entity on the routing path.
   */
  static void setMotion(boost::optional<double>& pAngularMotion,
                        boost::optional<double>& pEntryMotion, boost::optional<double>& pExitMotion,
                        double pRotationAngle,
                        const mars::agent::physical::common::RobotAgentProperties& pRAP,
                        Origin& pCurrentEntity,
                        Target& pNextEntity);

  static void setMotion(boost::optional<double>& pAngularMotion,
                        boost::optional<double>& pEntryMotion, boost::optional<double>& pExitMotion,
                        double pRotationAngle,
                        const mars::agent::physical::common::RobotAgentProperties& pRAP,
                        Origin& pCurrentEntity,
                        Target& pNextEntity,
                        Target& pPreviousEntity);

  /**
   * @brief setMotion Calculates angular and linear motions inside the resource in case the robot
   * agent is using a differential drive.
   * @param pAngularMotion Angular motion to calculate. Sets reference inside this function.
   * @param pEntryMotion Linear entry motion to calculate. Sets reference inside this function.
   * @param pExitMotion Linear exit motion to calculate. Sets reference inside this function.
   * @param pRotationAngle Required angle to rotate on the resource.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   * @param pPreviousEntity Previous topology entity on the routing path.
   */
  // static void setDifferentialMotion(boost::optional<double>& pAngularMotion,
  //                                   boost::optional<double>& pEntryMotion,
  //                                   boost::optional<double>& pExitMotion,
  //                                   double pRotationAngle,
  //                                   Origin& pCurrentEntity,
  //                                   Target& pNextEntity,
  //                                   Target& pPreviousEntity);

  /**
   * @brief setMotion Calculates scaled linear motions inside the resource in case the robot agent
   * is using an Ackermann drive. In this case there is no rotation on the spot and hence no angular
   * motion. Only slightly slower and longer linear motions based on the radius.
   * @param pEntryMotion Linear entry motion to calculate. Sets reference inside this function.
   * @param pExitMotion Linear exit motion to calculate. Sets reference inside this function.
   * @param pTurningRadius Turning radius inside the resource, restricted by the robots highest
   * possible turning angle.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   * @param pPreviousEntity Previous topology entity on the routing path.
   */
  static void setAckermannMotion(boost::optional<double>& pEntryMotion,
                                 boost::optional<double>& pExitMotion, double pTurningRadius,
                                 Origin& pCurrentEntity,
                                 Target& pNextEntity);

  static void setAckermannMotion(boost::optional<double>& pEntryMotion,
                                 boost::optional<double>& pExitMotion, double pTurningRadius,
                                 Origin& pCurrentEntity,
                                 Target& pNextEntity,
                                 Target& pPreviousEntity);

  /**
   * @brief setMotion Calculates estimated motion profiles for given angular and linear motions.
   * @param pRotationProfile Angular motion profile to calculate. Sets reference inside this
   * function.
   * @param pEntryProfile Linear entry motion profile to calculate. Sets reference inside this
   * function.
   * @param pExitProfile Linear exit motion profile to calculate. Sets reference inside this
   * function.
   * @param pAngularMotion Angle to rotate on the resource.
   * @param pEntryMotion Entry distance to the center of the resource.
   * @param pExitMotion Exit distance from the center of the resource.
   * @param pRAP Estimated robot motion parameters. Required for accurate time estimation. Can be
   * scaled or biased for more accuracy.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   * @param pVelocity Entry velocity into the resource. Calculated in previous iterations.
   */
  static void setProfile(boost::optional<Profile>& pRotationProfile,
                         boost::optional<Profile>& pEntryProfile,
                         boost::optional<Profile>& pExitProfile, double pAngularMotion,
                         double pEntryMotion, double pExitMotion,
                         const mars::agent::physical::common::RobotAgentProperties& pRAP,
                         Origin& pCurrentEntity,
                         Target& pNextEntity,
                         double pVelocity);
};
}  // namespace utility
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_UTILITY_TIMEPREDICTION_H
