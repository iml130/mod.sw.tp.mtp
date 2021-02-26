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

#ifndef MARS_AGENT_PHYSICAL_COMMON_ROBOTACTION_H
#define MARS_AGENT_PHYSICAL_COMMON_ROBOTACTION_H

#include <cstdint>
#include <string>

#include <mars_agent_physical_robot_msgs/RobotAction.h>

namespace mars
{
namespace agent
{
namespace physical
{
namespace common
{

/**
 * @brief Maps a category from e.g. a ros msg to a RobotActionCategory (enum).
 *
 */
enum RobotActionCategory
{
  UNDEFINED = 0,
  NONE = 5,
  LOAD = 10,
  UNLOAD = 20,
  START_CHARGING = 30,
  STOP_CHARGING = 31
};

/**
 * @brief Object of the ros msg mars_agent_physical_msgs/RobotAction.msg
 *
 */
class RobotAction
{
public:
  /**
   * @brief Construct a new RobotAction object. Throws a mars::common::exception::SetParamException
   * if the category can not be mapped to mars::agent::physical::common::RobotActionCategory.
   *
   * @param pCategory The category of the action. Must be mapable with RobotActionCategory!
   * @param pAction The robot specific action which should be performed.
   */
  RobotAction(uint8_t pCategory, uint8_t pAction);

  /**
   * @brief Construct a new RobotAction object. Throws a mars::common::exception::SetParamException
   * if the category can not be mapped to mars::agent::physical::common::RobotActionCategory.
   *
   * @param pCategory The category of the action. Must be mapable with RobotActionCategory!
   * @param pAction The specific action (for e.g. robot) which should be performed.
   * @param pDescription Human readable description of the action.
   */
  RobotAction(uint8_t pCategory, uint8_t pAction, std::string pDescription);

  /**
   * @brief Construct a new Robot Action object. Throws a mars::common::exception::SetParamException
   * if the category can not be mapped to mars::agent::physical::common::RobotActionCategory.
   *
   * @param pRobotActionMsg Is the RobotAction.msg
   */
  RobotAction(const mars_agent_physical_robot_msgs::RobotAction& pRobotActionMsg);

  /**
   * @brief Destroy the RobotAction object
   *
   */
  ~RobotAction();

  /**
   * @brief Get the Category object
   *
   * @return Returns the action category.
   */
  RobotActionCategory getCategory(void) const;

  /**
   * @brief Get the Category object
   *
   * @return Returns the action category.
   */
  uint8_t getCategoryRaw(void) const;

  /**
   * @brief Get the Action object
   *
   * @return Returns the action which has to be performed by a physical agent.
   */
  uint8_t getAction(void) const;

  /**
   * @brief Get the Description of an action. The action description describes the action in a human
   * readable way.
   *
   * @return Returns the description of an action.
   */
  std::string getDescription(void) const;

  mars_agent_physical_robot_msgs::RobotAction toMsg() const;

private:
  /**
   * @brief Describes the category of action which has to be performed.
   *
   */
  RobotActionCategory mCategory;

  /**
   * @brief Action code which has to be performed by a physical agent.
   *
   */
  uint8_t mAction;

  /**
   * @brief Optional description of the action
   *
   */
  std::string mDescription;

  /**
   * @brief Maps a given category to mars::agent::physical::common::RobotActionCategory.
   *
   * @param pCategory which should be maped.
   * @return Returns true if the category can be mapped to
   * mars::agent::physical::common::RobotActionCategory.
   */
  bool mapRobotActionCategory(uint8_t pCategory);
};
} // namespace common
} // namespace physical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_PHYSICAL_COMMON_ROBOTACTION_H