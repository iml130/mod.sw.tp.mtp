#include "mars_agent_physical_common/RobotAgentProperties.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/ConstantProfile.h"
#include "mars_routing_common/utility/MotionProfile.h"
#include "mars_routing_common/utility/differential.h"
#include "mars_routing_common/utility/timeprediction.h"

#include "mars_common/geometry/Segment.h"
#include "mars_routing_common/geometry/utilities.h"

#include <algorithm>
#include <stdlib.h>
#include <type_traits>

#include <boost/geometry/algorithms/intersection.hpp>
#include <eigen3/Eigen/Core>

template <class Origin, class Target, class Profile>
boost::optional<ros::Duration>
mars::routing::common::utility::timeprediction<Origin, Target, Profile>::getResourceDuration(
    Origin& pCurrentEntity, Target& pNextEntity, Target& pPreviousEntity,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
    mars::routing::common::utility::TimeInfo<Profile>& pTimeInfo)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  boost::optional<Profile> lRotationProfile;
  boost::optional<Profile> lEntryProfile;
  boost::optional<Profile> lExitProfile;
  boost::optional<double> lAngularMotion;
  boost::optional<double> lEntryMotion;
  boost::optional<double> lExitMotion;
  ros::Duration lRobotFrontEntryDuration;
  ros::Duration lRobotBackExitDuration;
  ros::Duration lRessourceDuration;

  boost::optional<Eigen::Vector3d> lBoostOptionalCurrentLocation = pCurrentEntity.getLocation();
  Eigen::Vector3d lCurrentLocation = lBoostOptionalCurrentLocation.value();
  boost::optional<Eigen::Vector3d> lBoostOptionalNextLocation = pNextEntity.getLocation();
  Eigen::Vector3d lNextLocation = lBoostOptionalNextLocation.value();
  boost::optional<Eigen::Vector3d> lBoostOptionalPreviousLocation = pPreviousEntity.getLocation();
  Eigen::Vector3d lPreviousLocation = lBoostOptionalPreviousLocation.value();


  if (!lBoostOptionalCurrentLocation || !lBoostOptionalNextLocation || !lBoostOptionalPreviousLocation)
  {
    return boost::none;
  }

  double lRotationAngle = mars::routing::common::geometry::utilities::get2DRotationAngle(
      lCurrentLocation, lNextLocation,
      mars::routing::common::geometry::utilities::get2DAngleBetweenPoints(lPreviousLocation,
                                                                          lCurrentLocation));

  mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setMotion(
      lAngularMotion, lEntryMotion, lExitMotion, lRotationAngle, pRAP, pCurrentEntity, pNextEntity,
      pPreviousEntity);

  if (!lAngularMotion || !lEntryMotion || !lExitMotion)
  {
    return boost::none;
  }

  mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setProfile(
      lRotationProfile, lEntryProfile, lExitProfile, *lAngularMotion, *lEntryMotion, *lExitMotion,
      pRAP, pCurrentEntity, pNextEntity, pVelocity);

  if (!lRotationProfile || !lEntryProfile || !lExitProfile)
  {
    return boost::none;
  }

  pVelocity = lExitProfile->getExitVelocity();
  double tmpEntryVelo = lEntryProfile->getExitVelocity();

  lRobotFrontEntryDuration =
      ros::Duration(pRAP.getFrontLength() / lEntryProfile->getEntryVelocity());
  lRobotBackExitDuration = ros::Duration(pRAP.getBackLength() / lExitProfile->getExitVelocity());

  lRessourceDuration = ros::Duration(lRobotFrontEntryDuration + lEntryProfile->getMotionDuration() +
                                     lRotationProfile->getMotionDuration() +
                                     lExitProfile->getMotionDuration() + lRobotBackExitDuration);

  pTimeInfo.setEntryMotion(lEntryMotion.value());
  pTimeInfo.setEntryProfile(lEntryProfile.value());
  pTimeInfo.setAngularMotion(lAngularMotion.value());
  pTimeInfo.setRotationProfile(lRotationProfile.value());
  pTimeInfo.setExitMotion(lExitMotion.value());
  pTimeInfo.setExitProfile(lExitProfile.value());

  return lRessourceDuration;
}

template <class Origin, class Target, class Profile>
boost::optional<ros::Duration>
mars::routing::common::utility::timeprediction<Origin, Target, Profile>::getResourceDuration(
    Origin& pCurrentEntity, Target& pNextEntity, double pOrientation,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
    mars::routing::common::utility::TimeInfo<Profile>& pTimeInfo)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  boost::optional<Profile> lRotationProfile;
  boost::optional<Profile> lEntryProfile;
  boost::optional<Profile> lExitProfile;
  boost::optional<double> lAngularMotion;
  boost::optional<double> lEntryMotion;
  boost::optional<double> lExitMotion;
  ros::Duration lRobotBackExitDuration;

  boost::optional<Eigen::Vector3d> lCurrentLocation = pCurrentEntity.getLocation();
  boost::optional<Eigen::Vector3d> lNextLocation = pNextEntity.getLocation();

  if (!lCurrentLocation || !lNextLocation)
  {
    return boost::none;
  }

  double lRotationAngle = mars::routing::common::geometry::utilities::get2DRotationAngle(
      *lCurrentLocation, *lNextLocation, pOrientation);

  mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setMotion(
      lAngularMotion, lEntryMotion, lExitMotion, lRotationAngle, pRAP, pCurrentEntity, pNextEntity);

  if (!lAngularMotion || !lExitMotion)
  {
    return boost::none;
  }

  mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setProfile(
      lRotationProfile, lEntryProfile, lExitProfile, *lAngularMotion, 0, *lExitMotion, pRAP,
      pCurrentEntity, pNextEntity, pVelocity);

  if (!lRotationProfile || !lEntryProfile || !lExitProfile)
  {
    return boost::none;
  }

  pVelocity = lExitProfile->getExitVelocity();

  lRobotBackExitDuration = ros::Duration(pRAP.getBackLength() / lExitProfile->getExitVelocity());

  //pTimeInfo.setEntryMotion(lEntryMotion.value());
 // pTimeInfo.setEntryProfile(lEntryProfile.value());
  pTimeInfo.setAngularMotion(lAngularMotion.value());
  pTimeInfo.setRotationProfile(lRotationProfile.value());
  pTimeInfo.setExitMotion(lExitMotion.value());
  pTimeInfo.setExitProfile(lExitProfile.value());

  return lEntryProfile->getMotionDuration() + lRotationProfile->getMotionDuration() +
         lExitProfile->getMotionDuration() + lRobotBackExitDuration;
}

template <class Origin, class Target, class Profile>
void mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setMotion(
    boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
    boost::optional<double>& pExitMotion, double pRotationAngle,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, Origin& pCurrentEntity,
    Target& pNextEntity)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  double lTurningRadius = pRAP.getTurningRadius();

  bool lAxialTurn = lTurningRadius == 0;
  if (lAxialTurn)
  {
    mars::routing::common::utility::differential<Origin, Target, Profile>::setMotion(
        pAngularMotion, pEntryMotion, pExitMotion, pRotationAngle, pCurrentEntity, pNextEntity);
  }
  else
  {
    pAngularMotion = 0;
    mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setAckermannMotion(
        pEntryMotion, pExitMotion, lTurningRadius, pCurrentEntity, pNextEntity);
  }
}

template <class Origin, class Target, class Profile>
void mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setMotion(
    boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
    boost::optional<double>& pExitMotion, double pRotationAngle,
    const mars::agent::physical::common::RobotAgentProperties& pRAP, Origin& pCurrentEntity,
    Target& pNextEntity, Target& pPreviousEntity)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  double lTurningRadius = pRAP.getTurningRadius();

  bool lAxialTurn = lTurningRadius == 0.0;
  if (lAxialTurn)
  {
    mars::routing::common::utility::differential<Origin, Target, Profile>::setMotion(
        pAngularMotion, pEntryMotion, pExitMotion, pRotationAngle, pCurrentEntity, pNextEntity,
        pPreviousEntity);
  }
  else
  {
    pAngularMotion = 0;
    mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setAckermannMotion(
        pEntryMotion, pExitMotion, lTurningRadius, pCurrentEntity, pNextEntity, pPreviousEntity);
  }
}

template <class Origin, class Target, class Profile>
void mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setAckermannMotion(
    boost::optional<double>& pEntryMotion, boost::optional<double>& pExitMotion,
    double pTurningRadius, Origin& pCurrentEntity, Target& pNextEntity)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  // TODO: Tangential circle calculation
  // For now no Ackermann calculations available.
  pEntryMotion = boost::none;
  pExitMotion = boost::none;
}

template <class Origin, class Target, class Profile>
void mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setAckermannMotion(
    boost::optional<double>& pEntryMotion, boost::optional<double>& pExitMotion,
    double pTurningRadius, Origin& pCurrentEntity, Target& pNextEntity, Target& pPreviousEntity)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  // TODO: Tangential circle calculation
  // For now no Ackermann calculations available.
  pEntryMotion = boost::none;
  pExitMotion = boost::none;
}

template <class Origin, class Target, class Profile>
void mars::routing::common::utility::timeprediction<Origin, Target, Profile>::setProfile(
    boost::optional<Profile>& pRotationProfile, boost::optional<Profile>& pEntryProfile,
    boost::optional<Profile>& pExitProfile, double pAngularMotion, double pEntryMotion,
    double pExitMotion, const mars::agent::physical::common::RobotAgentProperties& pRAP,
    Origin& pCurrentEntity, Target& pNextEntity, double pVelocity)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  boost::optional<mars::topology::common::TopologyEntityRestrictions&> lCurrentEntityRestrictions =
      pCurrentEntity.getRestrictions();
  boost::optional<mars::topology::common::TopologyEntityRestrictions&> lNextEntityRestrictions =
      pNextEntity.getRestrictions();

  if (!lCurrentEntityRestrictions || !lNextEntityRestrictions)
  {
    pRotationProfile = boost::none;
    pEntryProfile = boost::none;
    pExitProfile = boost::none;
    return;
  }

  double lLinearVelocity =
      std::min(pRAP.getForwardVelocity(), lCurrentEntityRestrictions->getMaxLinearVelocity());
  double lLinearAcceleration = std::min(pRAP.getLinearAcceleration(),
                                        lCurrentEntityRestrictions->getMaxLinearAcceleration());
  double lLinearDeceleration = std::min(pRAP.getLinearDeceleration(),
                                        lCurrentEntityRestrictions->getMaxLinearAcceleration());
  double lAngularVelocity =
      std::min(pRAP.getAngularVelocity(), lCurrentEntityRestrictions->getMaxAngularVelocity());
  double lAngularAcceleration = std::min(pRAP.getAngularAcceleration(),
                                         lCurrentEntityRestrictions->getMaxAngularAcceleration());
  double lAngularDeceleration = std::min(pRAP.getAngularDeceleration(),
                                         lCurrentEntityRestrictions->getMaxAngularAcceleration());

  double lExitVelocity =
      std::min(pRAP.getForwardVelocity(), lNextEntityRestrictions->getMaxLinearVelocity());

  pRotationProfile =
      Profile(pAngularMotion, 0, lAngularVelocity, 0, lAngularAcceleration, lAngularDeceleration);

  bool lNoRotation = pAngularMotion == 0;
  if (lNoRotation)
  {
    pEntryProfile = Profile(pEntryMotion, pVelocity, lLinearVelocity, lLinearVelocity,
                            lLinearAcceleration, lLinearDeceleration);
    pExitProfile = Profile(pExitMotion, pEntryProfile->getExitVelocity(), lLinearVelocity,
                           lExitVelocity, lLinearAcceleration, lLinearDeceleration);
  }
  else
  {
    pEntryProfile = Profile(pEntryMotion, pVelocity, lLinearVelocity, 0, lLinearAcceleration,
                            lLinearDeceleration);
    pExitProfile = Profile(pExitMotion, 0, lLinearVelocity, lExitVelocity, lLinearAcceleration,
                           lLinearDeceleration);
  }
}

template <class Origin, class Target, class Profile>
ros::Duration
mars::routing::common::utility::timeprediction<Origin, Target, Profile>::getMotionDuration(
    double pMotion, const mars::agent::physical::common::RobotAgentProperties& pRAP)
{
  static_assert(std::is_base_of<mars::routing::common::utility::MotionProfile, Profile>::value,
                "Profile type must "
                "extend MotionProfile");

  Profile lMotionProfile(pMotion, 0, pRAP.getForwardVelocity(), 0, pRAP.getLinearAcceleration(),
                         pRAP.getLinearDeceleration());

  return lMotionProfile.getMotionDuration();
}

template <class Origin, class Target, class Profile>
mars::routing::common::utility::timeprediction<Origin, Target, Profile>::timeprediction()
{
}

template class mars::routing::common::utility::timeprediction<
    mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge,
    mars::routing::common::utility::AffineProfile>;

template class mars::routing::common::utility::timeprediction<
    mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex,
    mars::routing::common::utility::AffineProfile>;
