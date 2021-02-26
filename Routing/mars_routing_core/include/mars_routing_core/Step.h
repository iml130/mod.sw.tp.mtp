#ifndef MARS_ROUTING_BASE_STEP_H
#define MARS_ROUTING_BASE_STEP_H

#include "mars_common/TimeInterval.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_common/utility/TimeInfo.h"
#include "mars_routing_msgs/Step.h"

#include <Eigen/Geometry>
#include <boost/optional.hpp>
#include <ros/time.h>

namespace mars
{
namespace routing
{
namespace core
{

/**
 * @brief Routing step with timed motion profile
 *
 * @tparam MotionProfile a motion profile
 */
template <typename MotionProfile> class TimedStep
{
public:
  virtual ~TimedStep(){};
  mars::routing::common::utility::TimeInfo<MotionProfile> getTimeInfo() const { return mTimeInfo; }

  void setTimeInfo(const mars::routing::common::utility::TimeInfo<MotionProfile>& pTimeInfo)
  {
    mTimeInfo = pTimeInfo;
  }

protected:
  mars::routing::common::utility::TimeInfo<MotionProfile> mTimeInfo;
};

/**
 * @class IterationStep
 */
class IterationStep
{
public:
  virtual ~IterationStep(){};
  virtual mars::routing::core::IterationStep* clone() const = 0;
  virtual mars_routing_msgs::Step toMsg() = 0;
  virtual mars::routing::common::topology::Entity& getOrigin() = 0;
  virtual mars::routing::common::topology::Entity& getTarget() = 0;
  virtual double getDistance() = 0;
  virtual const mars::common::TimeInterval& getTargetOccupationInterval() const = 0;
  virtual IterationStep* getPrevious() const = 0;
  virtual IterationStep* getNext() const = 0;
  virtual void nullifyPrevious() = 0;
  virtual void nullifyNext() = 0;
  virtual boost::optional<Eigen::Vector2d> getFootprintIntersection() = 0;
  virtual bool
  canDeallocateOrigin(const Eigen::Vector2d& pLocation,
                      const mars::agent::physical::common::RobotAgentProperties& pRAP) = 0;
};

/**
 * @class Step
 */
template <class Origin, class Target> class Step : public IterationStep
{
public:
  /**
   * @brief Default Constructor
   */
  Step(const mars_routing_msgs::Step& pStepMsg);

  Step(const mars::routing::core::Step<Origin, Target>& pStep);

  virtual ~Step();

  virtual mars::routing::core::Step<Origin, Target>* clone() const override;

  Origin& getOrigin() override;
  Target& getTarget() override;

  double getDistance();

  const mars::common::TimeInterval& getTargetOccupationInterval() const;

  Step<Target, Origin>* getPrevious() const override;
  Step<Target, Origin>* getNext() const override;

  void setTargetOccupationDuration(const ros::Duration& pOccupationDuration);

  void setPrevious(Step<Target, Origin>* pPrevious);
  void setNext(Step<Target, Origin>* pNext);

  void nullifyPrevious() override;
  void nullifyNext() override;

  mars_routing_msgs::Step toMsg() override;

  boost::optional<Eigen::Vector2d> getFootprintIntersection() override;

  bool
  canDeallocateOrigin(const Eigen::Vector2d& pLocation,
                      const mars::agent::physical::common::RobotAgentProperties& pRAP) override;

protected:
  Step(const Origin& pOrigin, const Target& pTarget,
       const mars::common::TimeInterval& pTargetOccupationInterval,
       Step<Target, Origin>* pPrevious = nullptr, Step<Target, Origin>* pNext = nullptr);

  Origin mOrigin;
  Target mTarget;

  mars::common::TimeInterval mTargetOccupationInterval;

  Step<Target, Origin>* mPrevious;
  Step<Target, Origin>* mNext;
};
} // namespace core
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_BASE_STEP_H
