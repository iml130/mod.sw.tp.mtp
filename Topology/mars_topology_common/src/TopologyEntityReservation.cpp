#include "mars_topology_common/TopologyEntityReservation.h"

#include <mars_common/TimeInterval.h>

static const std::string EXCEPTION_MSG_ALREADY_INITIALIZED =
    "TopologyEntityReservation is already "
    "initialized!";
static const std::string EXCEPTION_MSG_NOT_INITIALIZED =
    "TopologyEntityReservaltion was not "
    "initialized!";

mars::topology::common::TopologyEntityReservation::TopologyEntityReservation()
    : TimeIntervalBaseObject(
          mars::common::TimeInterval(ros::Time(), ros::Duration()))
{
  this->mAllocationGoal = boost::none;
  this->mIsInitialized = false;
}

mars::topology::common::TopologyEntityReservation::TopologyEntityReservation(
    const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
    const mars::common::TimeInterval& pReservationTime)
    : TimeIntervalBaseObject(pReservationTime)
{
  this->mAgentId = pAgentId;
  this->mPathId = pPathId;
  this->mAllocationGoal = boost::none;
  this->mIsInitialized = true;
}

mars::topology::common::TopologyEntityReservation::~TopologyEntityReservation()
{
}

void mars::topology::common::TopologyEntityReservation::initialize(
    const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
    const mars::common::TimeInterval& pReservationTime) noexcept(false)
{
  if (this->mIsInitialized)
  {
    throw mars::common::exception::SetParamException(
        EXCEPTION_MSG_ALREADY_INITIALIZED);
  }

  this->mAgentId = pAgentId;
  this->mPathId = pPathId;
  this->mTimeInterval = pReservationTime;
  this->mIsInitialized = true;
}

mars::common::Id
mars::topology::common::TopologyEntityReservation::getAgentId() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(
        EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mAgentId;
}

mars::common::Id
mars::topology::common::TopologyEntityReservation::getPathId() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(
        EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mPathId;
}

void mars::topology::common::TopologyEntityReservation::requestAllocation(
    const actionlib::ServerGoalHandle<
        mars_topology_actions::AllocateEntityAction>&
        pAllocationGoal) noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(
        EXCEPTION_MSG_NOT_INITIALIZED);
  }

  this->mAllocationGoal = pAllocationGoal;
  this->mAllocationGoal->setAccepted();
}

boost::optional<
    actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>>
mars::topology::common::TopologyEntityReservation::getAllocationGoal() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(
        EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mAllocationGoal;
}

bool mars::topology::common::TopologyEntityReservation::
operator<(const TopologyEntityReservation& pOtherReservation) const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(
        EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->getTimeInterval().getStartTime() <
         pOtherReservation.getTimeInterval().getStartTime();
}

bool mars::topology::common::TopologyEntityReservation::
operator<=(const TopologyEntityReservation& pOtherReservation) const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(
        EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->getTimeInterval().getStartTime() <=
         pOtherReservation.getTimeInterval().getStartTime();
}

std::vector<mars_topology_msgs::Reservation>
mars::topology::common::TopologyEntityReservation::convertToMsgReservation(
    const std::unordered_map<std::pair<mars::common::Id, mars::common::Id>,
                             std::vector<mars::topology::common::TopologyEntityReservation*>>& reservations)
{
  std::vector<mars_topology_msgs::Reservation> msgReservations;

  msgReservations.reserve(reservations.size());

  for (auto& iReservations : reservations)
  {
    for (auto& iReservation : iReservations.second)
    msgReservations.push_back(
        mars::topology::common::TopologyEntityReservation::convertToMsgReservation(
            *iReservation));
  }

  return msgReservations;
}

mars_topology_msgs::Reservation
mars::topology::common::TopologyEntityReservation::convertToMsgReservation(
    const mars::topology::common::TopologyEntityReservation& reservation)
{
  mars_topology_msgs::Reservation msgReservation;

  msgReservation.agent_id =
      mars::common::Id::convertToMsgId(reservation.getAgentId());
  msgReservation.path_id =
      mars::common::Id::convertToMsgId(reservation.getPathId());
  msgReservation.time_interval = reservation.getTimeInterval().toMsg();

  return msgReservation;
}

std::string
std::to_string(const mars::topology::common::TopologyEntityReservation&
                   pTopologyEntityReservation)
{
  if (!pTopologyEntityReservation.isInitialized())
  {
    return ("Not intialized!");
  }
  else
  {
    std::string str = "Agent id: " +
                      pTopologyEntityReservation.getAgentId().getUUIDAsString(
                          mars::common::Id::HEXDEC_SPLIT) +
                      "\n" + "Path id: " +
                      pTopologyEntityReservation.getPathId().getUUIDAsString(
                          mars::common::Id::HEXDEC_SPLIT) +
                      "\n" + "Timeinterval: " +
                      to_string(pTopologyEntityReservation.getTimeInterval());
    return str;
  }
}

std::string std::to_string(
    const shared_ptr<mars::topology::common::TopologyEntityReservation>&
        pTopologyEntityReservation)
{
  if (!pTopologyEntityReservation->isInitialized())
  {
    return ("Not intialized!");
  }
  else
  {
    std::string str =
        "Agent id: " +
        std::to_string(pTopologyEntityReservation->getAgentId()) + "\n" +
        "Path id: " + std::to_string(pTopologyEntityReservation->getPathId()) +
        "\n" + "Timeinterval: " +
        to_string(pTopologyEntityReservation->getTimeInterval());
    return str;
  }
}
