#include "mars_agent_logical_common/behavior/SetId.h"
#include "mars_common/Logger.h"

BT::NodeStatus mars::agent::logical::common::behavior::SetId::tick()
{
  BT::Optional<std::string> lUUIDString;
  BT::Result lResult;

  lUUIDString =
      this->getInput<std::string>(BEHAVIOR_SETID_PARAM_NAME_UUID_STRING);
  if (!lUUIDString)
  {
    MARS_LOG_ERROR(lUUIDString.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<mars::common::Id>(BEHAVIOR_SETID_PARAM_NAME_ID, mars::common::Id(lUUIDString.value()));
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}