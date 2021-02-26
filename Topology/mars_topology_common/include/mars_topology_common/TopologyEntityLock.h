#ifndef MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYLOCK_H
#define MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYLOCK_H

#include "TimeIntervalBasedObject.h"

#include <mars_common/Id.h>
#include <mars_common/TimeInterval.h>
#include <mars_common/exception/NotInitializedException.h>
#include <mars_common/exception/SetParamException.h>
#include <mars_topology_msgs/Lock.h>

#include <sstream>
#include <string>

#include <unordered_map>

namespace mars
{
namespace topology
{
namespace common
{
class TopologyEntityLock : public TimeIntervalBaseObject
{
public:
  TopologyEntityLock();

  /**
   * @brief LockRequest Creates an object of LockRequest.
   * @param initiatorId The id of an initiator who wants to lock the entity.
   * @param reason The reason to lock this entity.
   * @param startTime Start time of the lock.
   * @param duration Duration of the lock.
   */
  TopologyEntityLock(mars::common::Id initiatorId, std::string reason, ros::Time startTime, ros::Duration duration);

  /**
   * @brief LockRequest Creates an object of LockRequest.
   * @param initiatorId The id of an initiator who wants to lock the entity.
   * @param reason The reason to lock this entity.
   * @param timeInterval Time interval of the lock.
   */
  TopologyEntityLock(mars::common::Id initiatorId, std::string reason, mars::common::TimeInterval timeInterval);

  /**
   * @brief LockRequest Deletes a LockRequest object.
   */
  ~TopologyEntityLock();

  /**
   * @brief init Initializes an object of LockRequest.
   * @param initiatorId The id of an initiator who wants to lock the entity.
   * @param reason The reason to lock this entity.
   * @param startTime Start time of the lock.
   * @param duration Duration of the lock.
   * @throw mars::common::exception::SetParamException
   */
  void initialize(mars::common::Id initiatorId, std::string reason, ros::Time startTime,
                  ros::Duration duration) noexcept(false);

  /**
   * @brief init Initializes an object of LockRequest.
   * @param initiatorId The id of an initiator who wants to lock the entity.
   * @param reason The reason to lock this entity.
   * @param TimeInterval Time interval of the lock.
   * @throw mars::common::exception::SetParamException
   */
  void initialize(mars::common::Id initiatorId, std::string reason,
                  mars::common::TimeInterval timeInterval) noexcept(false);

  /**
   * @brief getInitiatorId The getter of the member variable mInitiatorId of the
   * class LockRequest.
   * @return mInitiatorId
   * @throw mars::common::exception::NotInitializedException
   */
  mars::common::Id getInitiatorId(void) const noexcept(false);
  /**
   * @brief getReason The getter of the member variable mReason of the class
   * LockRequest.
   * @return mReason
   * @throw mars::common::exception::NotInitializedException
   */
  std::string getReason(void) const noexcept(false);

  /**
   * @brief Get the Id object
   * 
   * @return mars::common::Id 
   * @throw mars::common::exception::NotInitializedException
   */
  mars::common::Id getId(void) const noexcept(false);

  bool isInitialized(void);

  bool operator==(const TopologyEntityLock& topologyEntityLock) const;

  static std::vector<mars_topology_msgs::Lock>
  convertToMsgLock(const std::unordered_map<mars::common::Id, mars::topology::common::TopologyEntityLock*>& locks);

  static mars_topology_msgs::Lock convertToMsgLock(const mars::topology::common::TopologyEntityLock& lock);

private:
  /**
   * @brief mInitiatorId The id of an initiator who wants to lock the entity.
   */
  mars::common::Id mInitiatorId;
  /**
   * @brief mReason the reason to lock this entity
   */
  std::string mLockReason;
  /**
   * @brief mId Id of the LockRequest object.
   */
  mars::common::Id mId;

  bool mIsInitialized;

  std::string generateDescription(void);
};
}  // namespace common
}  // namespace topology
}  // namespace mars

namespace std
{
inline string to_string(const mars::topology::common::TopologyEntityLock& val)
{
  stringstream ss;

  ss << "Lock Id: (" << to_string(val.getId()) << ")" << endl
     << "Description: " << val.getReason() << endl
     << "Initiator: (" << to_string(val.getInitiatorId()) << ")" << endl
     << "Time interval: (" << to_string(val.getTimeInterval()) << ")";

  return ss.str();
}
}

#endif  // MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYLOCK_H
