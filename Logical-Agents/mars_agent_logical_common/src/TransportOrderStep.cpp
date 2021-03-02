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

#include "mars_agent_logical_common/TransportOrderStep.h"

mars::agent::logical::common::TransportOrderStep::TransportOrderStep(
    const mars_agent_logical_msgs::TransportOrderStep& transportOrderStepMsg)
    : mars::agent::logical::common::Order(
          transportOrderStepMsg.transport_order_step_id), mMoveOrder(transportOrderStepMsg.move_order), mRobotAction(transportOrderStepMsg.robot_action)
{
}

std::shared_ptr<mars::routing::common::topology::Entity>
mars::agent::logical::common::TransportOrderStep::getDestination() const
{
  return this->mMoveOrder.getDestination();
}

const ros::Duration& mars::agent::logical::common::TransportOrderStep::
    getDestinationReservationDuration() const
{
  return this->mMoveOrder.getDestinationReservationDuration();
}

const mars::agent::logical::common::RobotAction& mars::agent::logical::common::TransportOrderStep::getRobotAction() const
{
  return mRobotAction;
}

const mars::agent::logical::common::MoveOrder& mars::agent::logical::common::TransportOrderStep::getMoveOrder() const
{
  return mMoveOrder;
}
