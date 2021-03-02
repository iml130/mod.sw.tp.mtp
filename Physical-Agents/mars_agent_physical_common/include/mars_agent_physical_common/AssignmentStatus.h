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


#ifndef MARS_AGENT_PHYSICAL_COMMON_ASSIGNMENTSTATUS_H
#define MARS_AGENT_PHYSICAL_COMMON_ASSIGNMENTSTATUS_H

#include <cstdint>
#include <string>

#include <geometry_msgs/PolygonStamped.h>

#include "mars_agent_physical_robot_msgs/AssignmentStatus.h"
#include "mars_common/Id.h"

namespace mars
{
namespace agent
{
namespace physical
{
namespace common
{
class AssignmentStatus
{
public:
  AssignmentStatus(const mars_common_msgs::Id pCurrentOrderId,
                   const mars_common_msgs::Id pCurrentMotionId,
                   const mars_common_msgs::Id pCurrentActionId,
                   const mars_common_msgs::Id pLastFinishedMotionId,
                   const mars_common_msgs::Id pLastFinishedActionId);

  AssignmentStatus(const mars_agent_physical_robot_msgs::AssignmentStatus& pAssignmentStatusMsg);

  ~AssignmentStatus();

  mars::common::Id getCurrentOrderId(void) const;

  mars::common::Id getCurrentMotionId(void) const;

  mars::common::Id getCurrentActionId(void) const;

  mars::common::Id getLastFinishedMotionId(void) const;

  mars::common::Id getLastFinishedActionId(void) const;

private:
  mars::common::Id mCurrentOrderId;
  mars::common::Id mCurrentMotionId;
  mars::common::Id mCurrentActionId;
  mars::common::Id mLastFinishedMotionId;
  mars::common::Id mLastFinishedActionId;
};
} // namespace common
} // namespace physical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_PHYSICAL_COMMON_ASSIGNMENTSTATUS_H
