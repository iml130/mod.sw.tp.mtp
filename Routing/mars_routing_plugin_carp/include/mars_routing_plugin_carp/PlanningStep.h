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

#ifndef MARS_ROUTING_PLUGIN_CARP_PLANNINGSTEP_H
#define MARS_ROUTING_PLUGIN_CARP_PLANNINGSTEP_H

#include "mars_agent_physical_common/RobotAgentProperties.h"

#include "mars_common/Id.h"
#include "mars_common/TimeInterval.h"

#include "mars_routing_common/topology/Entity.h"


namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
/**
 * @class PlanningStep
 * Wrapper class for polymorphic operations without template specifiers.
 */
class PlanningStep
{
public:
  virtual ~PlanningStep(){};
  virtual mars::routing::plugin::carp::PlanningStep* clone() const = 0;
  virtual mars::common::Id getOriginId() const = 0;
  virtual mars::common::Id getTargetId() const = 0;
  virtual bool operator<(const PlanningStep& pOtherStep) const = 0;
  virtual const mars::common::TimeInterval& getPlannedTargetOccupationInterval() const = 0;
  virtual const mars::common::TimeInterval& getTargetFreeInterval() const = 0;
  virtual const ros::Duration& getOriginMotionDuration() const = 0;
  virtual const ros::Duration& getDestinationMotionDuration() const = 0;

  virtual PlanningStep* getPlannedPrevious() const = 0;
  virtual PlanningStep* getPlannedNext() const = 0;

  virtual void setPlannedPrevious(const PlanningStep* pPrevious) = 0;
  virtual void setPlannedNext(const PlanningStep* pNext) = 0;

  virtual void setPlannedTargetOccupationDuration(const ros::Duration& pTargetOccupationDuration) = 0;

  virtual bool route(const mars::agent::physical::common::RobotAgentProperties& pRAP,
                     mars::routing::common::topology::Entity& pDestination, std::vector<PlanningStep*>& pOpenList,
                     std::vector<PlanningStep*>& pClosedList,
                     const ros::Duration& pDestinationReservationDuration) = 0;

protected:
private:
};
}  // namespace carp
}  // namespace plugin
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_PLUGIN_CARP_PLANNINGSTEP_H
