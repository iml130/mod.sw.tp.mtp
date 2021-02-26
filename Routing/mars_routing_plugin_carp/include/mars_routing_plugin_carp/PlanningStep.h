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
