#ifndef MARS_ROUTING_PLUGIN_CARP_STEP_H
#define MARS_ROUTING_PLUGIN_CARP_STEP_H

#include "mars_common/TimeInterval.h"

#include "mars_routing_core/Step.h"

#include "mars_routing_plugin_carp/PlanningStep.h"

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_common/topology/Vertex.h"
#include <mars_routing_common/utility/TimeInfo.h>
#include <mars_routing_common/utility/AffineProfile.h>

namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
/**
 * @class Step
 */
template <class Origin, class Target, class MotionProfile>
class Step : public mars::routing::plugin::carp::PlanningStep, public mars::routing::core::Step<Origin, Target>, public mars::routing::core::TimedStep<MotionProfile> 
{
public:
  Step(Origin& pOrigin, Target& pTarget, const mars::common::TimeInterval& pTargetOccupationInterval,
       const mars::common::TimeInterval& pTargetFreeInterval, const double& pEntryVelocity, const double& pExitVelocity,
       const ros::Duration& pOriginMotionDuration, const ros::Duration& pDestinationMotionDuration,
       Step<Target, Origin, MotionProfile>* pPrevious = nullptr, Step<Target, Origin, MotionProfile>* pNext = nullptr);

  mars::routing::plugin::carp::Step<Origin, Target, MotionProfile>* clone() const;

  mars::common::Id getOriginId() const;
  mars::common::Id getTargetId() const;

  const double& getEntryVelocity() const;

  const double& getExitVelocity() const;

  const ros::Duration& getOriginMotionDuration() const;

  const ros::Duration& getDestinationMotionDuration() const;

  bool operator<(const PlanningStep& pOtherStep) const;

  const mars::common::TimeInterval& getTargetFreeInterval() const;

  bool route(const mars::agent::physical::common::RobotAgentProperties& pRAP,
             mars::routing::common::topology::Entity& pDestination, std::vector<PlanningStep*>& pOpenList,
             std::vector<PlanningStep*>& pClosedList,
             const ros::Duration& pDestinationReservationDuration) override;

  const mars::common::TimeInterval& getPlannedTargetOccupationInterval() const;
  Step<Target, Origin, MotionProfile>* getPlannedPrevious() const;
  Step<Target, Origin, MotionProfile>* getPlannedNext() const;
  void setPlannedPrevious(const PlanningStep* pPrevious);
  void setPlannedNext(const PlanningStep* pNext);

  void setPlannedTargetOccupationDuration(const ros::Duration& pTargetOccupationDuration);


  // mars::routing::common::utility::TimeInfo<MotionProfile> getTimeInfo() const;
  // void setTimeInfo(const mars::routing::common::utility::TimeInfo<MotionProfile> &pTimeInfo);

private:
  Step();

  mars::common::TimeInterval mTargetFreeInterval;

  // mars::routing::common::utility::TimeInfo<MotionProfile> mTimeInfo;

  double mEntryVelocity;
  double mExitVelocity;
  
  ros::Duration mOriginMotionDuration;      /**< g costs */
  ros::Duration mDestinationMotionDuration; /**< h costs */
};
}  // namespace carp
}  // namespace plugin
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_PLUGIN_CARP_STEP_H
