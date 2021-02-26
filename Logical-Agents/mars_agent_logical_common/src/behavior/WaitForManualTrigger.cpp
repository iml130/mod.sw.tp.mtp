#include "mars_agent_logical_common/behavior/WaitForManualTrigger.h"

static const std::string SERVICE_ERROR_MSGS_NOT_TRIGGER_EXPECTED =
    "Received a trigger while not expecting one, it will be ignored.";

mars::agent::logical::common::behavior::WaitForManualTrigger::
    WaitForManualTrigger(const std::string& name,
                         const BT::NodeConfiguration& config)
    : BT::CoroActionNode(name, config)
{
  mWaitForTrigger = false;
  mTriggerReceived = false;
}

BT::NodeStatus
mars::agent::logical::common::behavior::WaitForManualTrigger::tick()
{
  this->mWaitForTrigger = true;

  while (!this->mTriggerReceived.load())
  {
    this->setStatusRunningAndYield();
  }

  // reset flags
  this->mWaitForTrigger = false;
  this->mTriggerReceived = false;

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList
mars::agent::logical::common::behavior::WaitForManualTrigger::providedPorts()
{
  // no ports provided
  return {};
}

bool mars::agent::logical::common::behavior::WaitForManualTrigger::
    serviceTriggerCallback(
        mars_agent_logical_srvs::ManualActionDoneRequest& pReq,
        mars_agent_logical_srvs::ManualActionDoneResponse& pRes)
{
  if (mWaitForTrigger.load())
  {
    mTriggerReceived = true;
  }
  else
  {
    pRes.result.result = mars_common_msgs::Result::RESULT_ERROR;
    pRes.result.description = SERVICE_ERROR_MSGS_NOT_TRIGGER_EXPECTED;
  }
  return true;
}

std::atomic<bool> mars::agent::logical::common::behavior::WaitForManualTrigger::
    mTriggerReceived;

std::atomic<bool> mars::agent::logical::common::behavior::WaitForManualTrigger::
    mWaitForTrigger;
