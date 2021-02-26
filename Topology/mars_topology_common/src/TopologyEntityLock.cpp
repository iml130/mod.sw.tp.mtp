#include "mars_topology_common/TopologyEntityLock.h"

#include <sstream>

mars::topology::common::TopologyEntityLock::TopologyEntityLock()
//: TimeIntervalBaseObject(mars::common::TimeInterval(ros::Time::now(), ros::Duration(1)))
{
  this->mIsInitialized = false;
}

mars::topology::common::TopologyEntityLock::TopologyEntityLock(mars::common::Id initiatorId, std::string reason,
                                                               ros::Time startTime, ros::Duration duration)
  : TimeIntervalBaseObject(mars::common::TimeInterval(startTime, duration))
{
  this->mId.initialize();
  this->mId.setDescription(this->generateDescription());
  this->mInitiatorId = initiatorId;
  this->mLockReason = reason;
  this->mIsInitialized = true;
}

mars::topology::common::TopologyEntityLock::TopologyEntityLock(mars::common::Id initiatorId, std::string reason,
                                                               mars::common::TimeInterval timeInterval)
  : TimeIntervalBaseObject(timeInterval)
{
  this->mId.initialize();
  this->mId.setDescription(this->generateDescription());
  this->mInitiatorId = initiatorId;
  this->mLockReason = reason;
  this->mIsInitialized = true;
}

mars::topology::common::TopologyEntityLock::~TopologyEntityLock()
{
}

void mars::topology::common::TopologyEntityLock::initialize(
    mars::common::Id initiatorId, std::string reason, ros::Time startTime,
    ros::Duration duration) noexcept(false)
{
  this->mTimeInterval.setStartTime(startTime);
  this->mTimeInterval.setDuration(duration);
  this->mId.initialize();
  this->mId.setDescription(this->generateDescription());
  this->mInitiatorId = initiatorId;
  this->mLockReason = reason;

  if (this->mTimeInterval.getEndTime() >= ros::Time::now())
  {
    this->mIsInitialized = true;
  }
  else
  {
    throw mars::common::exception::SetParamException("End time of the reservation is in the past.");
  }
}

void mars::topology::common::TopologyEntityLock::initialize(
    mars::common::Id initiatorId, std::string reason,
    mars::common::TimeInterval timeInterval) noexcept(false)
{
  this->mTimeInterval = timeInterval;
  this->mId.initialize();
  this->mId.setDescription(this->generateDescription());
  this->mInitiatorId = initiatorId;
  this->mLockReason = reason;

  if (this->mTimeInterval.getEndTime() >= ros::Time::now())
  {
    this->mIsInitialized = true;
  }
  else
  {
    throw mars::common::exception::SetParamException("End time of the reservation is in the past.");
  }
}

mars::common::Id mars::topology::common::TopologyEntityLock::getInitiatorId() const
    noexcept(false)
{
  if (this->mIsInitialized)
  {
    return this->mInitiatorId;
  }
  else
  {
    throw mars::common::exception::NotInitializedException("TopologyEntityLock was not initialized correctly!");
  }
}

std::string mars::topology::common::TopologyEntityLock::getReason() const
    noexcept(false)
{
  if (this->mIsInitialized)
  {
    return this->mLockReason;
  }
  else
  {
    throw mars::common::exception::NotInitializedException("TopologyEntityLock was not initialized correctly!");
  }
}

mars::common::Id mars::topology::common::TopologyEntityLock::getId() const
    noexcept(false)
{
  if (this->mIsInitialized)
  {
    return this->mId;
  }
  else
  {
    throw mars::common::exception::NotInitializedException("TopologyEntityLock was not initialized correctly!");
  }
}

bool mars::topology::common::TopologyEntityLock::isInitialized()
{
  return this->mIsInitialized;
}

bool mars::topology::common::TopologyEntityLock::operator==(const TopologyEntityLock& otherTopologyEntityLock) const
{
  return (this->mTimeInterval == otherTopologyEntityLock.getTimeInterval());
}

std::vector<mars_topology_msgs::Lock> mars::topology::common::TopologyEntityLock::convertToMsgLock(
    const std::unordered_map<mars::common::Id, mars::topology::common::TopologyEntityLock*>& locks)
{
  std::vector<mars_topology_msgs::Lock> msgLocks;

  msgLocks.reserve(locks.size());

  for (auto& iLock : locks)
  {
    msgLocks.push_back(mars::topology::common::TopologyEntityLock::convertToMsgLock(*iLock.second));
  }

  return msgLocks;
}

mars_topology_msgs::Lock
mars::topology::common::TopologyEntityLock::convertToMsgLock(const mars::topology::common::TopologyEntityLock& lock)
{
  mars_topology_msgs::Lock msgLock;

  msgLock.lock_id = mars::common::Id::convertToMsgId(lock.getId());
  msgLock.reason = lock.getReason();
  msgLock.time_interval = lock.getTimeInterval().toMsg();
  msgLock.lock_id = mars::common::Id::convertToMsgId(lock.getId());

  return msgLock;
}

std::string mars::topology::common::TopologyEntityLock::generateDescription()
{
  std::stringstream sstream;

  sstream << "TopologyEntityLock from " << this->mTimeInterval.getStartTime() << " to "
          << this->mTimeInterval.getEndTime();

  return sstream.str();
}
