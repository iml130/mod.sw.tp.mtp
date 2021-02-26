#include "mars_agent_logical_common/behavior/WaitForNSeconds.h"

static const std::string PARAM_NAME_WAIT_DURATION = "wait_duration";
static const std::string PARAM_DESCRIPTION_WAIT_DURATION =
    "Duration to wait until this node returns success.";

mars::agent::logical::common::behavior::WaitForNSeconds::WaitForNSeconds(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::CoroActionNode(name, config)
{
}

BT::NodeStatus mars::agent::logical::common::behavior::WaitForNSeconds::tick()
{
  // vom blackboard lesen oder aus ros launch ?
  ros::Duration waitDuration = getWaitDuration();

  ros::Time startTime = ros::Time::now();

  while ((ros::Time::now() - startTime) <= waitDuration)
  {
    this->setStatusRunningAndYield();
  }

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList
mars::agent::logical::common::behavior::WaitForNSeconds::providedPorts()
{
  return {BT::InputPort<double>(PARAM_NAME_WAIT_DURATION,
                                PARAM_DESCRIPTION_WAIT_DURATION)};
}

void mars::agent::logical::common::behavior::WaitForNSeconds::halt()
{
  std::cout << name() << ": Halted." << std::endl;

  // Do not forget to call this at the end.
  CoroActionNode::halt();
}

ros::Duration
mars::agent::logical::common::behavior::WaitForNSeconds::getWaitDuration()
{
  BT::Optional<double> waitDuration;

  waitDuration = this->getInput<double>(PARAM_NAME_WAIT_DURATION);
  if (!waitDuration)
  {
    MARS_LOG_ERROR(waitDuration.error());
    throw mars::common::exception::ReadParamException(
        "Could not read the waiting duration from the blackboard.");
  }
  else
  {
    return ros::Duration(waitDuration.value());
  }
}
