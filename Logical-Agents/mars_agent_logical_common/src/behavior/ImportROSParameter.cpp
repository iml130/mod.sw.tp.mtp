#include "mars_agent_logical_common/behavior/ImportROSParameter.h"
#include "mars_common/Logger.h"

static const std::string SERVICE_NAME_GET_ROUTE = "/GetRoute";

BT::NodeStatus mars::agent::logical::common::behavior::ImportROSParameter::tick()
{
  std::string lValue;
  BT::Optional<std::string> lParameterName;
  BT::Result lResult;

  lParameterName = this->getInput<std::string>(BEHAVIOR_IMPORTROSPARAMETER_PARAM_NAME);
  if (!lParameterName)
  {
    MARS_LOG_ERROR(lParameterName.error());
    return BT::NodeStatus::FAILURE;
  }

  if (!this->mNHPriv.getParam(lParameterName.value(), lValue))
  {
    MARS_LOG_ERROR_GET_ROS_PARAMETER(lParameterName.value());
    return BT::NodeStatus::FAILURE;
  };

  lResult = this->setOutput<std::string>(BEHAVIOR_IMPORTROSPARAMETER_PARAM_OUTPUT_KEY, lValue);
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}