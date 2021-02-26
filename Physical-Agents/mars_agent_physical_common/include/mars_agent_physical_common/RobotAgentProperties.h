#include "mars_agent_physical_robot_msgs/RobotAgentProperties.h"
#include "mars_agent_physical_robot_msgs/VehicleType.h"
#include "mars_common/geometry/Footprint.h"
#include "mars_topology_common/TopologyEntityRestrictions.h"
#include "mars_topology_msgs/HazardType.h"

#ifndef MARS_ROUTING_COMMON_ROBOTAGENTPROPERTIES_H
#define MARS_ROUTING_COMMON_ROBOTAGENTPROPERTIES_H

namespace mars
{
namespace agent
{
namespace physical
{
namespace common
{
/**
 * @class RobotAgentProperties
 * @brief Container class for robot agent motion parameters.
 */
class RobotAgentProperties
{
public:
  /**
   * @brief Default constructor for const attributes.
   * @param pId Unique ID of the robot containing a UUID and a description.
   * @param pVehicleType The vehicle type of the robot agent.
   * @param pFootprint The footprint of the robot agent.
   * @param pForwardVelocity Maximum linear velocity targeted by the robot during forward motion.
   * Can be lower than what the robot is truly capable of.
   * @param pBackwardVelocity Maximum linear velocity targeted by the robot during backward motion.
   * Can be lower than what the robot is truly capable of.
   * @param pLinearAcceleration Estimated inear acceleration value of the robot. May be downscaled
   * or biased for better simulated motions based on simplified models.
   * @param pLinearDeceleration Estimated inear deceleration value of the robot. May be downscaled
   * or biased for better simulated motions based on simplified models.
   * @param pAngularVelocity Maximum rotational velocity around base axis targeted by the robot. Can
   * be lower than what the robot is truly capable of.
   * @param pAngularAcceleration Estimated rotational acceleration value of the robot. May be
   * downscaled or biased for better simulated motions based on simplified models.
   * @param pAngularDeceleration Estimated rotational deceleration value of the robot. May be
   * downscaled or biased for better simulated motions based on simplified models.
   * @param pMinHeight The height of the vehicle in minimal extension state.
   * @param pMaxHeight The height of the vehicle in maximal extension state, e.g. extended scissor
   * lift.
   * @param pWeight The weight of the vehicle excluding any payload.
   * @param pPayload Maximum possible payload the robot can carry.
   * @param pTurningRadius Least possible turning radius of the robot during linear motion. 0 for
   * robots, which can only turn on the spot.
   */
  RobotAgentProperties(const mars::common::Id& pId,
                       const mars_agent_physical_robot_msgs::VehicleType& pVehicleType,
                       const mars::common::geometry::Footprint& pFootprint,
                       double pForwardVelocity, double pBackwardVelocity,
                       double pLinearAcceleration, double pLinearDeceleration,
                       double pAngularVelocity, double pAngularAcceleration,
                       double pAngularDeceleration, double pMinHeight, double pMaxHeight,
                       double pWeight, double pPayload, double pTurningRadius = 0);

  /**
   * @brief Direct constructor for robot description.
   * @param pRobotAgentProperties Parameters of the robot.
   */
  RobotAgentProperties(
      const mars_agent_physical_robot_msgs::RobotAgentProperties& pRobotAgentProperties);

  /**
   * @brief Matches properties against topology entity restrictions.
   * Disregards the HazardType.
   *
   * @param pTopologyEntityRestrictions Set of restrictions on the topology entity.
   * @return true, if robot agent with these properties can move on the topology entity with the
   * given restrictions.
   * @return false otherwise.
   */
  bool match(
      const mars::topology::common::TopologyEntityRestrictions& pTopologyEntityRestrictions) const;

  /**
   * @brief Matches properties and hazard type against topology entity restrictions.
   *
   * @param pTopologyEntityRestrictions Set of restrictions on the topology entity.
   * @param pHazardType Set of hazards possibly loaded on the robot.
   * @return true, if robot agent with these properties can move on the topology entity with the
   * given restrictions.
   * @return false otherwise.
   */
  bool match(const mars::topology::common::TopologyEntityRestrictions& pTopologyEntityRestrictions,
             const mars_topology_msgs::HazardType& pHazardType) const;

  /**
   * @brief Primitive getter for mVehicleType.
   * @return The vehicle type of the robot agent.
   */
  const mars_agent_physical_robot_msgs::VehicleType& getVehicleType() const;

  /**
   * @brief Primitive getter for mFootprint.
   * @return The footprint of the robot agent.
   */
  const mars::common::geometry::Footprint& getFootprint() const;

  /**
   * @brief Primitive getter for mForwardVelocity.
   * @return Current maximum targeted forward velocity.
   */
  double getForwardVelocity() const;

  /**
   * @brief Primitive getter for mBackwardVelocity.
   * @return Current maximum targeted backward velocity.
   */
  double getBackwardVelocity() const;

  /**
   * @brief Primitive getter for mLinearAcceleration.
   * @return Current maximum linear acceleration. May be scaled to bias real acceleration for better
   * estimations.
   */
  double getLinearAcceleration() const;

  /**
   * @brief Primitive getter for mLinearDeceleration.
   * @return Current maximum linear deceleration upon braking. May be scaled to bias real
   * deceleration for better estimations.
   */
  double getLinearDeceleration() const;

  /**
   * @brief Primitive getter for mAngularVelocity.
   * @return Current maximum targeted angular velocity.
   */
  double getAngularVelocity() const;

  /**
   * @brief Primitive getter for mAngularAcceleration.
   * @return Current maximum angular acceleration. May be scaled to bias real acceleration for
   * better
   * estimations. May be 0 for certain drive types.
   */
  double getAngularAcceleration() const;

  /**
   * @brief Primitive getter for mAngularDeceleration.
   * @return Current maximum angular deceleration. May be scaled to bias real deceleration for
   * better
   * estimations. May be 0 for certain drive types.
   */
  double getAngularDeceleration() const;

  /**
   * @brief Primitive getter for mMinHeight.
   * @return Height of the robot in a minimal state.
   */
  double getMinHeight() const;

  /**
   * @brief Primitive getter for mMaxHeight.
   * @return Height of the robot in a fully extended state, e.g. extended scissor lift.
   */
  double getMaxHeight() const;

  /**
   * @brief Primitive getter for mWeight.
   * @return Weight of the robot without any additional payload.
   */
  double getWeight() const;

  /**
   * @brief Primitive getter for mPayload.
   * @return Maximal allowed weight of the load the robot can carry.
   */
  double getPayload() const;

  /**
   * @brief Getter for maximal distance to footprint keypoints.
   * @return Radius at which the vehicle can safely turn on the spot.
   */
  double getCollisionRadius() const;

  /**
   * @brief Simple getter for the smallest possible turning radius.
   * @return Calculated based on turning angle. return Current smallest possible turning radius. May
   * be 0
   * for certain drive types.
   */
  double getTurningRadius() const;

  mars_agent_physical_robot_msgs::RobotAgentProperties toMsg() const;

  double getFrontLength() const;

  double getBackLength() const;

protected:
  mars::common::Id mId;
  mars_agent_physical_robot_msgs::VehicleType mVehicleType;
  mars::common::geometry::Footprint mFootprint;

  double mForwardVelocity;
  double mBackwardVelocity;
  double mLinearAcceleration;
  double mLinearDeceleration;
  double mAngularVelocity;
  double mAngularAcceleration;
  double mAngularDeceleration;
  double mMinHeight;
  double mMaxHeight;
  double mWeight;
  double mPayload;
  double mTurningRadius;
};
} // namespace common
} // namespace physical
} // namespace agent
} // namespace mars

#endif // MARS_ROUTING_COMMON_ROBOTAGENTPROPERTIES_H
