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
