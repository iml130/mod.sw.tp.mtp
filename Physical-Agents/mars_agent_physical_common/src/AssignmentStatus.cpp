/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : L4MS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

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
