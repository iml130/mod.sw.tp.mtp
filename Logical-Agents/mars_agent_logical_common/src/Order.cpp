/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : MARS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#include <mars_agent_logical_common/Order.h>

#include <string>

mars::agent::logical::common::Order::Order(const mars::common::Id& pId)
    : mId(pId), mCreationTime(0), mStartTime(0), mCompletionTime(0)
{
}

mars::agent::logical::common::Order::~Order() {}

mars::common::Id mars::agent::logical::common::Order::getId() const { return this->mId; }

ros::Time mars::agent::logical::common::Order::getCreationTime() const
{
  return this->mCreationTime;
}

ros::Time mars::agent::logical::common::Order::getStartTime() const { return this->mCreationTime; }

ros::Time mars::agent::logical::common::Order::getCompletionTime() const
{
  return this->mCompletionTime;
}

bool mars::agent::logical::common::Order::isCompleted() const
{
  return this->mCompletionTime == ros::Time(0);
}