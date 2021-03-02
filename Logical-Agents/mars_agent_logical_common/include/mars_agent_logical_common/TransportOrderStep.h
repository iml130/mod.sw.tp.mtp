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

#ifndef TRANSPORTORDERSTEP_H
#define TRANSPORTORDERSTEP_H

#include "mars_agent_logical_common/MoveOrder.h"
#include "mars_agent_logical_common/Order.h"
#include "mars_agent_logical_common/RobotAction.h"
#include <mars_agent_logical_msgs/TransportOrderStep.h>
#include <mars_common/Id.h>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
class TransportOrderStep : public mars::agent::logical::common::Order
{
public:
  TransportOrderStep(
      const mars_agent_logical_msgs::TransportOrderStep& transportOrderStepMsg);

  std::shared_ptr<mars::routing::common::topology::Entity>
  getDestination() const;

  const ros::Duration& getDestinationReservationDuration() const;

  const mars::agent::logical::common::MoveOrder& getMoveOrder() const;

  const mars::agent::logical::common::RobotAction& getRobotAction() const;

private:
  /**
   * @brief mMoveOrder Movement part of the transport order step
   */
  mars::agent::logical::common::MoveOrder mMoveOrder;

  mars::agent::logical::common::RobotAction mRobotAction;
};

} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // TRANSPORTORDERSTEP_H
