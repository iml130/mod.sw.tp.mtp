#ifndef MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYRESERVATION_H
#define MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYRESERVATION_H

#include "TimeIntervalBasedObject.h"

#include <mars_common/Id.h>
#include <mars_common/TimeInterval.h>
#include <mars_common/exception/NotInitializedException.h>
#include <mars_common/exception/SetParamException.h>
#include <mars_topology_actions/AllocateEntityAction.h>
#include <mars_topology_msgs/Reservation.h>

#include <actionlib/server/action_server.h>

#include <string>
#include <unordered_map>

namespace mars
{
namespace topology
{
namespace common
{
class TopologyEntityReservation : public TimeIntervalBaseObject
{
public:
  /**
   * @brief TopologyEntityReservation Creats a new object of the
   */
  TopologyEntityReservation(void);

  TopologyEntityReservation(const mars::common::Id& pAgentId,
                            const mars::common::Id& pPathId,
                            const mars::common::TimeInterval& pReservationTime);

  ~TopologyEntityReservation();

  /**
   * @brief
   *
   * @param pAgentId
   * @param pPathId
   * @param pReservationTime
   * @throw mars::common::exception::SetParamException
   */
  void initialize(
      const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
      const mars::common::TimeInterval& pReservationTime) noexcept(false);

  /**
   * @brief Get the Agent Id object
   *
   * @return mars::common::Id
   * @throw mars::common::exception::NotInitializedException
   */
  mars::common::Id getAgentId(void) const noexcept(false);

  /**
   * @brief Get the Path Id object
   *
   * @return mars::common::Id
   * @throw mars::common::exception::NotInitializedException
   */
  mars::common::Id getPathId(void) const noexcept(false);

  /**
   * @brief
   *
   * @param pAllocationGoal
   * @throw mars::common::exception::NotInitializedException
   */
  void requestAllocation(const actionlib::ServerGoalHandle<
                         mars_topology_actions::AllocateEntityAction>&
                             pAllocationGoal) noexcept(false);

  /**
   * @brief Get the Allocation Goal object
   *
   * @return
   * boost::optional<actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>>
   * @throw mars::common::exception::NotInitializedException
   */
  boost::optional<
      actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>>
  getAllocationGoal() const noexcept(false);

  /**
   * @brief
   *
   * @param pOtherReservation
   * @return true
   * @return false
   * @throw mars::common::exception::NotInitializedException
   */
  bool operator<(const TopologyEntityReservation& pOtherReservation) const
      noexcept(false);

  /**
   * @brief
   *
   * @param pOtherReservation
   * @return true
   * @return false
   * @throw mars::common::exception::NotInitializedException
   */
  bool operator<=(const TopologyEntityReservation& pOtherReservation) const
      noexcept(false);

  static std::vector<mars_topology_msgs::Reservation> convertToMsgReservation(
      const std::unordered_map<std::pair<mars::common::Id, mars::common::Id>,
                              std::vector<mars::topology::common::TopologyEntityReservation*>>& reservations);

  static mars_topology_msgs::Reservation convertToMsgReservation(
      const mars::topology::common::TopologyEntityReservation& reservation);

private:
  /**
   * @brief mAgentId Id of the reserving agent.
   */
  mars::common::Id mAgentId;

  /**
   * @brief mPathId Id of the path for which the reservation is.
   */
  mars::common::Id mPathId;

  /**
   * @brief mIsInitialized True, if the TopologyEntityReservation is
   * initialized.
   */
  bool mIsInitialized;

  boost::optional<
      actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>>
      mAllocationGoal;
};
} // namespace common
} // namespace topology
} // namespace mars

namespace std
{
 std::string
to_string(const shared_ptr<mars::topology::common::TopologyEntityReservation>&
              pTopologyEntityReservation);

std::string
to_string(const mars::topology::common::TopologyEntityReservation&
                   pTopologyEntityReservation);
} // namespace std

#endif // MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYRESERVATION_H
