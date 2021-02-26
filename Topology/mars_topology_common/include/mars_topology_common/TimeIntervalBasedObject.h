#ifndef MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECT_H
#define MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECT_H

#include <mars_common/TimeInterval.h>
#include <mars_common/exception/SetParamException.h>
#include <mars_common/exception/NotInitializedException.h>

#include <memory>

namespace mars
{
namespace topology
{
namespace common
{
class TimeIntervalBaseObject
{
public:
  TimeIntervalBaseObject() : mTimeInterval(ros::Time::now(), ros::Duration(1))
  {
    this->mIsInitialized = false;
  }

  virtual ~TimeIntervalBaseObject() {};

  /**
   * @brief Construct a new Time Interval Base Object object
   * 
   * @param pTimeInterval 
   * @throw mars::common::exception::SetParamException
   */
  TimeIntervalBaseObject(mars::common::TimeInterval pTimeInterval) noexcept(false)
      : mTimeInterval(pTimeInterval)
  {
    this->mIsInitialized = true;
  }

  void setTimeInterval(mars::common::TimeInterval pTimeInterval)
  {
    this->mIsInitialized = true;
    this->mTimeInterval = pTimeInterval;
  }

  /**
   * @brief Get the Time Interval object
   * 
   * @return mars::common::TimeInterval 
   * @throw mars::common::exception::NotInitializedException
   */
  virtual mars::common::TimeInterval getTimeInterval() const
      noexcept(false)
  {
    if (!this->mIsInitialized)
    {
      throw mars::common::exception::NotInitializedException("TimeInterval was not initialized!");
    }
    return this->mTimeInterval;
  }

  virtual bool isInitialized(void) const { return this->mIsInitialized; }


protected:
  mars::common::TimeInterval mTimeInterval;

private:
  bool mIsInitialized;
};
} // namespace common
} // namespace topology
} // namespace mars

namespace std
{
inline string
to_string(const shared_ptr<mars::topology::common::TimeIntervalBaseObject>& pTimeIntervalBaseObject)
{
  if (!pTimeIntervalBaseObject->isInitialized())
  {
    return ("Not intialized!");
  }
  else
  {
    return to_string(pTimeIntervalBaseObject->getTimeInterval());
  }
}

inline string
to_string(const mars::topology::common::TimeIntervalBaseObject& pTimeIntervalBaseObject)
{
  if (!pTimeIntervalBaseObject.isInitialized())
  {
    return ("Not intialized!");
  }
  else
  {
    return to_string(pTimeIntervalBaseObject.getTimeInterval());
  }
}
}

#endif // MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECT_H
