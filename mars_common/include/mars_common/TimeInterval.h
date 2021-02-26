#ifndef MARS_COMMON_TIMEINTERVAL_H
#define MARS_COMMON_TIMEINTERVAL_H

#include "mars_common/exception/SetParamException.h"

#include <ros/time.h>
#include <mars_topology_msgs/TimeInterval.h>

#include <string>
#include <sstream>
#include <stdint.h>

namespace mars
{
namespace common
{
class TimeInterval
{
public:
  static const ros::Duration INFINITE_DURATION;

  static const ros::Time INFINITE_END_TIME;

  /**
   * @brief TimeInterval Constructs a object of this class. For creating an infinite time interval,
   * set the start time you want and for duration use the INFINITE_DURATION constant!
   * @param time Start time
   * @param duration Duration of the Interval
   * @throw exception::SetParamException
   */
  TimeInterval(const ros::Time& time, const ros::Duration& duration) noexcept(false);

  /**
   * @brief TimeInterval Constructs a object of this class.
   * @param time Start time
   * @param duration Duration of the Interval
   */
  TimeInterval(const ros::Time& startTime, const ros::Time& endTime);

  /**
   * @brief TimeInterval Constructs a object of this class from a corresponding ROS message.
   * @param pTimeIntervalMessage ROS message to convert from.
   */
  TimeInterval(const mars_topology_msgs::TimeInterval& pTimeIntervalMessage);

  virtual ~TimeInterval();

  /**
   * @brief setStartTime Sets the starttime
   * @param time Starttime
   */
  void setStartTime(const ros::Time& time);

  /**
   * @brief setDuration Sets the duration
   * @param duration Duration of
   * @throw exception::SetParamException
   */
  void setDuration(const ros::Duration& duration) noexcept(false);

  /**
   * @brief getStartTime Gets the start time of the interval
   * @return The start time of this interval
   */
  ros::Time getStartTime() const;

  /**
   * @brief getStartTime Gets the end time of the interval
   * @return The end time of this interval
   */
  ros::Time getEndTime() const;

  /**
   * @brief getDuration Gets the duration of the interval
   * @return The duration of this interval
   */
  ros::Duration getDuration() const;

  bool operator==(const TimeInterval& otherTimeInterval) const;

  bool operator<(const TimeInterval& otherTimeInterval) const;

  bool operator<=(const TimeInterval& otherTimeInterval) const;

  bool operator>(const TimeInterval& otherTimeInterval) const;

  bool operator>=(const TimeInterval& otherTimeInterval) const;

  bool operator!=(const TimeInterval& otherTimeInterval) const;

  mars_topology_msgs::TimeInterval toMsg() const;

private:
  /**
   * @brief mTime Start time of this Timeinterval.
   */
  ros::Time mStartTime;

  /**
   * @brief mDuration Duration beginning from the start time. A duration < 0 is
   * considered to be infinite.
   */
  ros::Duration mDuration;

  /**
   * @brief mEndTime End time of the time interval.
   */
  ros::Time mEndTime;
};
} // namespace common
} // namespace mars

namespace std
{
inline string to_string(const mars::common::TimeInterval& val)
{
  stringstream ss;

  ss << "Time interval from " << val.getStartTime() << " to " << val.getEndTime()
     << " (duration: " << val.getDuration() << ")";

  return ss.str();
}
}

#endif // MARS_COMMON_TIMEINTERVAL_H
