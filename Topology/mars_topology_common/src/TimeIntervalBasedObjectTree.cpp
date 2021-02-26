#include "mars_topology_common/TimeIntervalBasedObjectTree.h"

mars::topology::common::TimeIntervalBasedObjectTree::TimeIntervalBasedObjectTree()
{
  this->mRoot = NULL;
}

void mars::topology::common::TimeIntervalBasedObjectTree::insert(
    mars::topology::common::TimeIntervalBaseObject*
        timeIntervalBaseObject) noexcept(false)
{
  this->mRoot = this->insertTimeIntervalBaseObject(this->mRoot, timeIntervalBaseObject);
}

bool mars::topology::common::TimeIntervalBasedObjectTree::checkOverlap(
    const mars::common::TimeInterval& pTimeInterval) const
{
  return (this->checkOverlapWithTree(this->mRoot, pTimeInterval) != NULL);
}

std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>
mars::topology::common::TimeIntervalBasedObjectTree::getRoot() const
{
  return this->mRoot;
}

std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>
mars::topology::common::TimeIntervalBasedObjectTree::insertTimeIntervalBaseObject(
    std::shared_ptr<TimeIntervalBasedObjectTreeNode> pRoot,
    mars::topology::common::TimeIntervalBaseObject* pTimeIntervalBaseObject)
{
  // Base case: Tree is empty, new node becomes root
  if (pRoot == NULL)
  {
    //    return std::make_shared<mars::topology::common::TimeIntervalBasedObjectTreeNode>(
    //        pTimeIntervalBaseObject);

    return std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode>(
        new mars::topology::common::TimeIntervalBasedObjectTreeNode(pTimeIntervalBaseObject));
  }

  // If root's low value is smaller, then new interval goes to left subtree
  if (pTimeIntervalBaseObject->getTimeInterval().getStartTime() >
      pRoot->getTimeIntervalBaseObject()->getTimeInterval().getStartTime())
  {
    pRoot->setLeftNode(
        mars::topology::common::TimeIntervalBasedObjectTree::insertTimeIntervalBaseObject(
            pRoot->getLeftNode(), pTimeIntervalBaseObject));
  }
  // else, new node goes to right subtree.
  else
  {
    pRoot->setRightNode(
        mars::topology::common::TimeIntervalBasedObjectTree::insertTimeIntervalBaseObject(
            pRoot->getRightNode(), pTimeIntervalBaseObject));
  }

  // Update the max value of this ancestor if needed
  if (pRoot->getMaxTime() < pTimeIntervalBaseObject->getTimeInterval().getEndTime())
  {
    pRoot->setMaxTime(pTimeIntervalBaseObject->getTimeInterval().getEndTime());
  }

  return pRoot;
}

std::shared_ptr<mars::topology::common::TimeIntervalBaseObject>
mars::topology::common::TimeIntervalBasedObjectTree::checkOverlapWithTree(
    std::shared_ptr<mars::topology::common::TimeIntervalBasedObjectTreeNode> pRoot,
    const mars::topology::common::TimeIntervalBaseObject& pTimeIntervalBaseObject)
{
  // Base Case, tree is empty
  if (pRoot == NULL)
    return NULL;

  // If given interval overlaps with root
  if (pRoot->getTimeIntervalBaseObject()->getTimeInterval() !=
      pTimeIntervalBaseObject.getTimeInterval())
  {
    return pRoot->getTimeIntervalBaseObject();
  }

  // If left child of root is present and max of left child is
  // greater than or equal to given interval, then i may
  // overlap with an interval is left subtree
  if ((pRoot->getLeftNode() != NULL) && (pRoot->getLeftNode()->getMaxTime() >=
                                         pTimeIntervalBaseObject.getTimeInterval().getStartTime()))
  {
    return mars::topology::common::TimeIntervalBasedObjectTree::checkOverlapWithTree(
        pRoot->getLeftNode(), pTimeIntervalBaseObject);
  }

  // Else interval can only overlap with right subtree
  return mars::topology::common::TimeIntervalBasedObjectTree::checkOverlapWithTree(
      pRoot->getRightNode(), pTimeIntervalBaseObject);
}
