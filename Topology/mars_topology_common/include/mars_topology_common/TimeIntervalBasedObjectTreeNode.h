#ifndef MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECTTREENODE_H
#define MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECTTREENODE_H

#include "TimeIntervalBasedObject.h"

#include <memory>
#include <iostream>
#include <sstream>

namespace mars
{
namespace topology
{
namespace common
{
/**
 * Binary tree node for TimeIntervalBasedObjects.
 */
class TimeIntervalBasedObjectTreeNode
{
public:
  TimeIntervalBasedObjectTreeNode(TimeIntervalBaseObject* pTimeIntervalBaseObject);

  TimeIntervalBasedObjectTreeNode(const TimeIntervalBaseObject& pTimeIntervalBaseObject);

  void setLeftNode(std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>
                       pTimeIntervalBasedObjectTreeNode);

  void setRightNode(std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>
                        pTimeIntervalBasedObjectTreeNode);

  void setMaxTime(ros::Time pMaxTime);

  std::shared_ptr<TimeIntervalBaseObject> getTimeIntervalBaseObject(void) const;

  std::shared_ptr<TimeIntervalBasedObjectTreeNode> getLeftNode(void) const;

  std::shared_ptr<TimeIntervalBasedObjectTreeNode> getRightNode(void) const;

  ros::Time getMaxTime(void) const;

private:
  std::shared_ptr<TimeIntervalBaseObject> mTimeIntervalBaseObject;

  ros::Time mMaxTime;

  std::shared_ptr<TimeIntervalBasedObjectTreeNode> mLeftNode;

  std::shared_ptr<TimeIntervalBasedObjectTreeNode> mRightNode;
};
} // namespace common
} // namespace topology
} // namespace mars

namespace std
{

inline string
to_string(const std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>&
              pTimeIntervalBasedObjectTreeNode)
{
  stringstream ss;

  if (pTimeIntervalBasedObjectTreeNode == NULL)
  {
    return "";
  }

  ss << to_string(pTimeIntervalBasedObjectTreeNode->getLeftNode()) << "["
     << to_string(pTimeIntervalBasedObjectTreeNode->getTimeIntervalBaseObject())
     << "] max = " << pTimeIntervalBasedObjectTreeNode->getMaxTime() << endl
     << to_string(pTimeIntervalBasedObjectTreeNode->getRightNode());

  return ss.str();
}

inline string to_string(
    const mars::topology::common::TimeIntervalBasedObjectTreeNode& pTimeIntervalBasedObjectTreeNode)
{
  stringstream ss;

  ss << to_string(pTimeIntervalBasedObjectTreeNode.getLeftNode()) << "["
     << to_string(pTimeIntervalBasedObjectTreeNode.getTimeIntervalBaseObject())
     << "] max = " << pTimeIntervalBasedObjectTreeNode.getMaxTime() << endl
     << to_string(pTimeIntervalBasedObjectTreeNode.getRightNode());

  return ss.str();
}
}

#endif // MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECTTREENODE_H
