//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

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
