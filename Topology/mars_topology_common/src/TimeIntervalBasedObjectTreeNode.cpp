#include "mars_topology_common/TimeIntervalBasedObjectTreeNode.h"
mars::topology::common::TimeIntervalBasedObjectTreeNode::TimeIntervalBasedObjectTreeNode(
    mars::topology::common::TimeIntervalBaseObject* pTimeIntervalBaseObject)
{
  this->mTimeIntervalBaseObject =
      std::shared_ptr<mars::topology::common::TimeIntervalBaseObject>(pTimeIntervalBaseObject);
  this->mLeftNode = NULL;
  this->mRightNode = NULL;
}

mars::topology::common::TimeIntervalBasedObjectTreeNode::TimeIntervalBasedObjectTreeNode(
    const mars::topology::common::TimeIntervalBaseObject& pTimeIntRervalBaseObject)
{
  this->mTimeIntervalBaseObject =
      std::make_shared<mars::topology::common::TimeIntervalBaseObject>(pTimeIntRervalBaseObject);
  this->mLeftNode = NULL;
  this->mRightNode = NULL;
}

void mars::topology::common::TimeIntervalBasedObjectTreeNode::setLeftNode(std::shared_ptr<
    mars::topology::common::TimeIntervalBasedObjectTreeNode> pTimeIntervalBasedObjectTreeNode)
{
  this->mLeftNode = pTimeIntervalBasedObjectTreeNode;
}

void mars::topology::common::TimeIntervalBasedObjectTreeNode::setRightNode(std::shared_ptr<
    mars::topology::common::TimeIntervalBasedObjectTreeNode> pTimeIntervalBasedObjectTreeNode)
{
  this->mRightNode = pTimeIntervalBasedObjectTreeNode;
}

void mars::topology::common::TimeIntervalBasedObjectTreeNode::setMaxTime(ros::Time pMaxTime)
{
  this->mMaxTime = pMaxTime;
}

std::shared_ptr<mars::topology::common::TimeIntervalBaseObject>
mars::topology::common::TimeIntervalBasedObjectTreeNode::getTimeIntervalBaseObject() const
{
  return this->mTimeIntervalBaseObject;
}

std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>
mars::topology::common::TimeIntervalBasedObjectTreeNode::getLeftNode() const
{
  return this->mLeftNode;
}

std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>
mars::topology::common::TimeIntervalBasedObjectTreeNode::getRightNode() const
{
  return this->mRightNode;
}

ros::Time mars::topology::common::TimeIntervalBasedObjectTreeNode::getMaxTime() const
{
  return this->mMaxTime;
}
