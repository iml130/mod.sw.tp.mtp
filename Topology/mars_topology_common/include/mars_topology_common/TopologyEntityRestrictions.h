#ifndef MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYRESTRICTIONS_H
#define MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYRESTRICTIONS_H

#include "mars_topology_msgs/HazardType.h"
#include "mars_agent_physical_robot_msgs/VehicleType.h"
#include "mars_topology_srvs/GetRestrictions.h"

#include "mars_common/Id.h"
#include "mars_common/TimeInterval.h"
#include "mars_common/exception/NotInitializedException.h"
#include "mars_common/exception/SetParamException.h"

namespace mars
{
namespace topology
{
namespace common
{
class TopologyEntityRestrictions
{
public:
  /*
   * @brief TopologyEntityRestrictions Default constructor for uninitialized (!) instance.
   * Avoid using this whenever possible.
   */
  TopologyEntityRestrictions(void);

  /*
   * @brief TopologyEntityRestrictions Constructor for initialized instance.
   * @param pMaxLinearVelocity Maximum linear velocity allowed on the topology entity. Robot agent
   * can throttle to match this velocity.
   * @param pMaxAngularVelocity Maximum angular velocity allowed on the topology entity. Robot agent
   * can throttle to match this velocity.
   * @param pMaxLinearAcceleration Maximum linear acceleration allowed on the topology entity. Robot
   * agent can throttle to match this acceleration.
   * @param pMaxAngularAcceleration Maximum angular acceleration allowed on the topology entity.
   * Robot agent can throttle to match this acceleration.
   * @param pMaxTotalWeight Maximum total weight allowed on the topology entity.
   * @param pMaxHeight Maximum height of an agent on the topology entity.
   * @param pForbiddenHazardTypes Set of hazard classifications, which are not allowed on the
   * topology entity.
   * @param pForbiddenVehicleTypes Set of vehicle types, which are not allowed on the topology
   * entity.
   */
  TopologyEntityRestrictions(
      const double& pMaxLinearVelocity, const double& pMaxAngularVelocity,
      const double& pMaxLinearAcceleration, const double& pMaxAngularAcceleration,
      const double& pMaxTotalWeight, const double& pMaxHeight,
      const std::vector<mars_topology_msgs::HazardType>& pForbiddenHazardTypes,
      const std::vector<mars_agent_physical_robot_msgs::VehicleType>& pForbiddenVehicleTypes);

  /*
   * @brief TopologyEntityRestrictions Constructor for initialized instance.
   * @param pServiceResponse Service response to use to directly initialize the instance.
   */
  TopologyEntityRestrictions(const mars_topology_srvs::GetRestrictionsResponse& pServiceResponse);

  /*
   * @brief ~TopologyEntityRestrictions Default destructor.
   * Does nothing for now.
   */
  ~TopologyEntityRestrictions();

  /*
   * @brief Initializer to validate the instance.
   * @param pMaxLinearVelocity Maximum linear velocity allowed on the topology entity. Robot agent
   * can throttle to match this velocity.
   * @param pMaxAngularVelocity Maximum angular velocity allowed on the topology entity. Robot agent
   * can throttle to match this velocity.
   * @param pMaxLinearAcceleration Maximum linear acceleration allowed on the topology entity. Robot
   * agent can throttle to match this acceleration.
   * @param pMaxAngularAcceleration Maximum angular acceleration allowed on the topology entity.
   * Robot agent can throttle to match this acceleration.
   * @param pMaxTotalWeight Maximum total weight allowed on the topology entity.
   * @param pMaxHeight Maximum height of an agent on the topology entity.
   * @param pForbiddenHazardTypes Set of hazard classifications, which are not allowed on the
   * topology entity.
   * @param pForbiddenVehicleTypes Set of vehicle types, which are not allowed on the topology
   * entity.
   * @throw mars::common::exception::SetParamException
   */
  void initialize(const double& pMaxLinearVelocity, const double& pMaxAngularVelocity,
                  const double& pMaxLinearAcceleration, const double& pMaxAngularAcceleration,
                  const double& pMaxTotalWeight, const double& pMaxHeight,
                  const std::vector<mars_topology_msgs::HazardType>& pForbiddenHazardTypes,
                  const std::vector<mars_agent_physical_robot_msgs::VehicleType>&
                      pForbiddenVehicleTypes) noexcept(false);

  /*
   * @brief Initializer to validate the instance.
   * @param pServiceResponse Service response to use to copy the instance attributes from.
   * @throw mars::common::exception::SetParamException
   */
  void initialize(const mars_topology_srvs::GetRestrictionsResponse& pServiceResponse) noexcept(false);

  /*
   * @brief Merges instance attributes into a service response.
   * @param pServiceResponse Service response to merge instance attributes into.
   * @throw mars::common::exception::NotInitializedException
   */
  void mergeIntoServiceResponse(mars_topology_srvs::GetRestrictionsResponse& pServiceResponse) const
      noexcept(false);

  /*
   * @brief Primitive getter for mMaxLinearVelocity.
   * @return Maximum linear velocity allowed on the topology entity. Robot agent can throttle to
   * match this velocity.
   * @throw mars::common::exception::NotInitializedException
   */
  double getMaxLinearVelocity(void) const noexcept(false);

  /*
   * @brief Primitive getter for mMaxAngularVelocity.
   * @return Maximum angular velocity allowed on the topology entity. Robot agent can throttle to
   * match this velocity.
   * @throw mars::common::exception::NotInitializedException
   */
  double getMaxAngularVelocity(void) const noexcept(false);

  /*
   * @brief Primitive getter for mMaxLinearAcceleration.
   * @return Maximum linear acceleration allowed on the topology entity. Robot agent can throttle to
   * match this acceleration.
   * @throw mars::common::exception::NotInitializedException
   */
  double getMaxLinearAcceleration(void) const noexcept(false);

  /*
   * @brief Primitive getter for mMaxAngularAcceleration.
   * @return Maximum angular acceleration allowed on the topology entity. Robot agent can throttle
   * to match this acceleration.
   * @throw mars::common::exception::NotInitializedException
   */
  double getMaxAngularAcceleration(void) const noexcept(false);

  /*
   * @brief Primitive getter for mMaxTotalWeight.
   * @return Maximum total weight allowed on the topology entity.
   * @throw mars::common::exception::NotInitializedException
   */
  double getMaxTotalWeight(void) const noexcept(false);

  /*
   * @brief Primitive getter for mMaxHeight.
   * @return Maximum height of an agent on the topology entity.
   * @throw mars::common::exception::NotInitializedException
   */
  double getMaxHeight(void) const noexcept(false);

  /*
   * @brief Primitive getter for mForbiddenHazardTypes.
   * @return Set of hazard classifications, which are not allowed on the topology entity.
   * @throw mars::common::exception::NotInitializedException
   */
  const std::vector<mars_topology_msgs::HazardType>& getForbiddenHazardTypes(void) const
      noexcept(false);

  /*
   * @brief Primitive getter for mForbiddenVehicleTypes.
   * @return Set of vehicle types, which are not allowed on the topology entity.
   * @throw mars::common::exception::NotInitializedException
   */
  const std::vector<mars_agent_physical_robot_msgs::VehicleType>&
  getForbiddenVehicleTypes(void) const noexcept(false);

private:
  double mMaxLinearVelocity;      /**< Maximum linear velocity allowed on this topology entity. */
  double mMaxAngularVelocity;     /**< Maximum angular velocity allowed on this topology entity. */
  double mMaxLinearAcceleration;  /**< Maximum linear acceleration allowed on this topology entity.
                                   */
  double mMaxAngularAcceleration; /**< Maximum angular acceleration allowed on this topology entity.
                                   */

  double mMaxTotalWeight; /**< Maximum total weight allowed on this topology entity. */
  double mMaxHeight;      /**< Maximum height of an agent on this topology entity. */

  std::vector<mars_topology_msgs::HazardType> mForbiddenHazardTypes; /**< Hazard types, which are
                                                                        not allowed on this
                                                                        topology entity. */
  std::vector<mars_agent_physical_robot_msgs::VehicleType>
      mForbiddenVehicleTypes; /**< Vehicle types, which are not allowed on this topology entity. */

  bool mIsInitialized = false;
};
}  // namespace common
}  // namespace topology
}  // namespace mars

#endif  // MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYRESTRICTIONS_H