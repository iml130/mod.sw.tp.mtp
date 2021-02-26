#include "mars_agent_logical_common/behavior/SetDuration.h"

static const std::string BEHAVIOR_SETDURATION_PARAM_NAME_RESERVATION_DURATION =
    "reservation_duration";
static const std::string BEHAVIOR_SETDURATION_PARAM_NAME_DURATION = "duration";

mars::agent::logical::common::behavior::SetDuration::SetDuration(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::SetDuration::providedPorts()
{
  {
    return {
        BT::InputPort< double>(BEHAVIOR_SETDURATION_PARAM_NAME_DURATION),
        BT::OutputPort<ros::Duration>(
            BEHAVIOR_SETDURATION_PARAM_NAME_RESERVATION_DURATION)};
  }
}

BT::NodeStatus mars::agent::logical::common::behavior::SetDuration::tick()
{
  BT::Optional< double> duration;
  BT::Result lResult;

  duration =
      this->getInput< double>(BEHAVIOR_SETDURATION_PARAM_NAME_DURATION);
  if (!duration)
  {
    MARS_LOG_ERROR(duration.error());
  }

  lResult = this->setOutput<ros::Duration>(
      BEHAVIOR_SETDURATION_PARAM_NAME_RESERVATION_DURATION,
      ros::Duration(duration.value()));
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
