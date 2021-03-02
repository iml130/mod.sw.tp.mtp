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


#include "mars_agent_physical_common/AssignmentStatus.h"

mars::agent::physical::common::AssignmentStatus::AssignmentStatus(const mars_common_msgs::Id pCurrentOrderId,
                                                                  const mars_common_msgs::Id pCurrentMotionId,
                                                                  const mars_common_msgs::Id pCurrentActionId,
                                                                  const mars_common_msgs::Id pLastFinishedMotionId,
                                                                  const mars_common_msgs::Id pLastFinishedActionId)
  : mCurrentOrderId(pCurrentOrderId)
  , mCurrentMotionId(pCurrentMotionId)
  , mCurrentActionId(pCurrentActionId)
  , mLastFinishedMotionId(pLastFinishedMotionId)
  , mLastFinishedActionId(pLastFinishedActionId)
{
}

mars::agent::physical::common::AssignmentStatus::AssignmentStatus(
    const mars_agent_physical_robot_msgs::AssignmentStatus& pAssignmentStatusMsg)
  : mCurrentOrderId(pAssignmentStatusMsg.current_task_id)
  , mCurrentMotionId(pAssignmentStatusMsg.current_motion_id)
  , mCurrentActionId(pAssignmentStatusMsg.current_action_id)
  , mLastFinishedMotionId(pAssignmentStatusMsg.last_finished_motion)
  , mLastFinishedActionId(pAssignmentStatusMsg.last_finished_action)
{
}

mars::agent::physical::common::AssignmentStatus::~AssignmentStatus()
{
}

mars::common::Id mars::agent::physical::common::AssignmentStatus::getCurrentOrderId() const
{
  return this->mCurrentOrderId;
}

mars::common::Id mars::agent::physical::common::AssignmentStatus::getCurrentMotionId() const
{
  return this->mCurrentMotionId;
}

mars::common::Id mars::agent::physical::common::AssignmentStatus::getCurrentActionId() const
{
  return this->mCurrentActionId;
}

mars::common::Id mars::agent::physical::common::AssignmentStatus::getLastFinishedMotionId() const
{
  return this->mLastFinishedMotionId;
}

mars::common::Id mars::agent::physical::common::AssignmentStatus::getLastFinishedActionId() const
{
  return this->mLastFinishedActionId;
}
