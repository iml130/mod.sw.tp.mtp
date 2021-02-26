#include "mars_agent_logical_common/behavior/StringToBool.h"

BT::NodeStatus mars::agent::logical::common::behavior::StringToBool::tick()
{
  BT::Optional<std::string> lBoolString;
  BT::Result lResult;

  lBoolString = this->getInput<std::string>(BEHAVIOR_STRINGTOBOOL_PARAM_NAME_STRING);
  if (!lBoolString)
  {
    MARS_LOG_ERROR(lBoolString.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<bool>(BEHAVIOR_STRINGTOBOOL_PARAM_NAME_BOOL,
                                  BT::convertFromString<bool>(lBoolString.value()));
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}