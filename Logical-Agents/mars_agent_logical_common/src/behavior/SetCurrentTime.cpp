#include "mars_agent_logical_common/behavior/SetCurrentTime.h"
#include "mars_common/Logger.h"

BT::NodeStatus mars::agent::logical::common::behavior::SetCurrentTime::tick()
{
  BT::Result lResult;

  lResult = this->setOutput<ros::Time>(BEHAVIOR_SETCURRENTTIME_PARAM_NAME_TIME, ros::Time::now());
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}