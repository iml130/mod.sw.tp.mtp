#include "mars_agent_logical_common/RobotAction.h"

mars::agent::logical::common::RobotAction::RobotAction(
    const mars_agent_physical_robot_msgs::RobotAction pRobotActionMsg)
    : mActionCategory(pRobotActionMsg.category),
      mAction(pRobotActionMsg.action), mDescription(pRobotActionMsg.description)
{
}

bool mars::agent::logical::common::RobotAction::isManual() const
{
  if (this->mActionCategory ==
          mars_agent_physical_robot_msgs::RobotAction::CATEGORY_MANUAL_LOAD ||
      this->mActionCategory ==
          mars_agent_physical_robot_msgs::RobotAction::CATEGORY_MANUAL_UNLOAD)
  {
    return true;
  }
  else
  {
    return false;
  }
}

const std::string
mars::agent::logical::common::RobotAction::getDescription() const
{
  return mDescription;
}

const std::vector<mars_common_msgs::Tuple>
mars::agent::logical::common::RobotAction::getAttributes() const
{
  return mAttributes;
}

const unsigned int mars::agent::logical::common::RobotAction::getAction() const
{
  return mAction;
}

const unsigned int mars::agent::logical::common::RobotAction::getCategory() const
{
  return mActionCategory;
}

void mars::agent::logical::common::RobotAction::initActionEnum()
{
  this->mActionEnums.insert(
      {{"CATEGORY_UNDEFINED",
        mars_agent_physical_robot_msgs::RobotAction::CATEGORY_UNDEFINED},
       {"CATEGORY_NONE",
        mars_agent_physical_robot_msgs::RobotAction::CATEGORY_NONE},
       {"CATEGORY_LOAD",
        mars_agent_physical_robot_msgs::RobotAction::CATEGORY_LOAD},
       {"CATEGORY_MANUAL_LOAD_START",
        mars_agent_physical_robot_msgs::RobotAction::
            CATEGORY_MANUAL_LOAD_START},
       {"CAREGORY_MANUAL_LOAD_DONE",
        mars_agent_physical_robot_msgs::RobotAction::CATEGORY_MANUAL_LOAD_DONE},
       {"CATEGORY_UNLOAD",
        mars_agent_physical_robot_msgs::RobotAction::CATEGORY_UNLOAD},
       {"CATEGORY_MANUAL_UNLOAD_START",
        mars_agent_physical_robot_msgs::RobotAction::
            CATEGORY_MANUAL_UNLOAD_START},
       {"CATEGORY_MANUAL_UNLOAD_DONE",
        mars_agent_physical_robot_msgs::RobotAction::
            CATEGORY_MANUAL_UNLOAD_DONE},
       {"CATEGORY_START_CHARGING",
        mars_agent_physical_robot_msgs::RobotAction::CATEGORY_START_CHARGING},
       {"CATEGORY_STOP_CHARGING",
        mars_agent_physical_robot_msgs::RobotAction::CATEGORY_STOP_CHARGING}});
}
