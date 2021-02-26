#include "mars_common/TimeInterval.h"

static const ros::Duration MIN_VALID_DURATION = ros::Duration(0, 0);

const ros::Duration mars::common::TimeInterval::INFINITE_DURATION = ros::Duration(-1, 0);

const ros::Time mars::common::TimeInterval::INFINITE_END_TIME = ros::Time(INT32_MAX, INT32_MAX);

static const std::string EXCEPTION_MSG_INVALID_DURATION =
    "Invalid duration for time interval was given, duration must "
    "be > 0!";

mars::common::TimeInterval::TimeInterval(const ros::Time& time,
                                         const ros::Duration& duration) noexcept(false)
{
  if (duration >= MIN_VALID_DURATION)
  {
    this->mStartTime = time;
    this->mDuration = duration;
    this->mEndTime = time + duration;
  }
  else if (duration == INFINITE_DURATION)
  {
    this->mStartTime = time;
    this->mDuration = duration;
    this->mEndTime = INFINITE_END_TIME;
  }
  else
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_INVALID_DURATION);
  }
}

mars::common::TimeInterval::TimeInterval(const ros::Time& startTime, const ros::Time& endTime)
    : TimeInterval(startTime, endTime - startTime)
{
}

mars::common::TimeInterval::TimeInterval(
    const mars_topology_msgs::TimeInterval& pTimeIntervalMessage)
    : TimeInterval(ros::Time(pTimeIntervalMessage.start_time),
                   ros::Duration(pTimeIntervalMessage.duration))
{
}

mars::common::TimeInterval::~TimeInterval() {}

void mars::common::TimeInterval::setStartTime(const ros::Time& time) { this->mStartTime = time; }

void mars::common::TimeInterval::setDuration(const ros::Duration& duration) noexcept(false)
{
if (duration >= MIN_VALID_DURATION)
  {
    this->mDuration = duration;
    this->mEndTime = this->mStartTime + duration;
  }
  else if (duration == INFINITE_DURATION)
  {
    this->mDuration = duration;
    this->mEndTime = INFINITE_END_TIME;
  }
  else
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_INVALID_DURATION);
  }
}

ros::Time mars::common::TimeInterval::getStartTime() const { return this->mStartTime; }

ros::Time mars::common::TimeInterval::getEndTime() const { return this->mEndTime; }

ros::Duration mars::common::TimeInterval::getDuration() const { return this->mDuration; }

bool mars::common::TimeInterval::operator==(const TimeInterval& otherTimeInterval) const
{
  return ((this->mStartTime == otherTimeInterval.getStartTime()) &&
          (this->getEndTime() == otherTimeInterval.getEndTime()));
}

bool mars::common::TimeInterval::operator<(const TimeInterval& otherTimeInterval) const
{
  return (this->getEndTime() < otherTimeInterval.getStartTime());
}

bool mars::common::TimeInterval::operator<=(const TimeInterval& otherTimeInterval) const
{
  return (this->getEndTime() <= otherTimeInterval.getStartTime());
}

bool mars::common::TimeInterval::operator>(const TimeInterval& otherTimeInterval) const
{
  return (this->getStartTime() > otherTimeInterval.getEndTime());
}

bool mars::common::TimeInterval::operator>=(const TimeInterval& otherTimeInterval) const
{
  return (this->getStartTime() >= otherTimeInterval.getEndTime());
}

bool mars::common::TimeInterval::operator!=(const TimeInterval& otherTimeInterval) const
{
  return ((this->mStartTime <= otherTimeInterval.getStartTime()) ||
          (this->getEndTime() >= otherTimeInterval.getEndTime()));
}

mars_topology_msgs::TimeInterval mars::common::TimeInterval::toMsg() const
{
  mars_topology_msgs::TimeInterval lTimeIntervalMsg;

  lTimeIntervalMsg.start_time = mStartTime;
  lTimeIntervalMsg.duration = mDuration;

  return lTimeIntervalMsg;
}
