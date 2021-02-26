/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : L4MS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#include "mars_agent_physical_common/RobotAction.h"

#include <mars_common/exception/SetParamException.h>

#include <sstream>

static const std::string MSG_UNKNOW_CATEGORY_EXCEPTION =
    "Given category can't be mapped to mars::agent::physical::common::RobotActionCategory";

mars::agent::physical::common::RobotAction::RobotAction(uint8_t pCategory, uint8_t pAction)
{
  if (this->mapRobotActionCategory(pCategory))
  {
    this->mAction = pAction;
  }
  else
  {
    std::stringstream ss;

    ss.str("");
    ss << MSG_UNKNOW_CATEGORY_EXCEPTION << ": " << pCategory;

    throw mars::common::exception::SetParamException(ss.str());
  }
}

mars::agent::physical::common::RobotAction::RobotAction(uint8_t pCategory, uint8_t pAction,
                                                        std::string pDescription)
{
  if (this->mapRobotActionCategory(pCategory))
  {
    this->mAction = pAction;
    this->mDescription = pDescription;
  }
}

mars::agent::physical::common::RobotAction::RobotAction(
    const mars_agent_physical_robot_msgs::RobotAction& pRobotActionMsg)
{
  if (this->mapRobotActionCategory(pRobotActionMsg.category))
  {
    this->mAction = pRobotActionMsg.action;
    this->mDescription = pRobotActionMsg.description;
  }
}

mars::agent::physical::common::RobotAction::~RobotAction() {}

mars::agent::physical::common::RobotActionCategory
mars::agent::physical::common::RobotAction::getCategory(void) const
{
  return this->mCategory;
}

uint8_t mars::agent::physical::common::RobotAction::getCategoryRaw() const
{
  return ((uint8_t)this->mCategory);
}

uint8_t mars::agent::physical::common::RobotAction::getAction() const { return this->mAction; }

std::string mars::agent::physical::common::RobotAction::getDescription() const
{
  return this->mDescription;
}

bool mars::agent::physical::common::RobotAction::mapRobotActionCategory(uint8_t pCategory)
{
  bool lMappedSuccessfully = true;

  switch (pCategory)
  {
  case mars::agent::physical::common::RobotActionCategory::NONE:
    this->mCategory = mars::agent::physical::common::RobotActionCategory::NONE;
    break;
  case mars::agent::physical::common::RobotActionCategory::LOAD:
    this->mCategory = mars::agent::physical::common::RobotActionCategory::LOAD;
    break;
  case mars::agent::physical::common::RobotActionCategory::UNLOAD:
    this->mCategory = mars::agent::physical::common::RobotActionCategory::UNLOAD;
    break;
  case mars::agent::physical::common::RobotActionCategory::START_CHARGING:
    this->mCategory = mars::agent::physical::common::RobotActionCategory::START_CHARGING;
    break;
  case mars::agent::physical::common::RobotActionCategory::STOP_CHARGING:
    this->mCategory = mars::agent::physical::common::RobotActionCategory::STOP_CHARGING;
    break;
  default:
    this->mCategory = mars::agent::physical::common::RobotActionCategory::UNDEFINED;
    lMappedSuccessfully = false;
    break;
  }

  return lMappedSuccessfully;
}

mars_agent_physical_robot_msgs::RobotAction
mars::agent::physical::common::RobotAction::toMsg() const
{
  mars_agent_physical_robot_msgs::RobotAction lRobotActionMsg;

  lRobotActionMsg.category = this->getCategoryRaw();
  lRobotActionMsg.action = this->getAction();
  lRobotActionMsg.description = this->getDescription();

  return lRobotActionMsg;
}