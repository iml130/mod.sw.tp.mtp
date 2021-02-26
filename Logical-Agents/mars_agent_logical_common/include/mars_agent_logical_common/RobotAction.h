#ifndef ROBOTACTION_H
#define ROBOTACTION_H

#include <mars_agent_physical_robot_msgs/RobotAction.h>

#include <unordered_map>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
class RobotAction
{
public:
  RobotAction(const mars_agent_physical_robot_msgs::RobotAction robotActionMsg);

  bool isManual() const;

  const unsigned int getCategory() const;

  const unsigned int getAction() const;

  const std::vector<mars_common_msgs::Tuple> getAttributes() const;

  const std::string getDescription() const;

private:

  unsigned int mActionCategory;

  unsigned int mAction;

  std::vector<mars_common_msgs::Tuple> mAttributes;

  std::string mDescription;

  std::unordered_map<std::string, unsigned int> mActionEnums;

  void initActionEnum();

};
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // ROBOTACTION_H
