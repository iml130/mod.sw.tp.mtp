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