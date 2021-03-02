//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

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
