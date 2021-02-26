#ifndef PUBLISHMOTIONASSIGNMENTSTOTARGET_H
#define PUBLISHMOTIONASSIGNMENTSTOTARGET_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <mars_common/Id.h>
#include <mars_common/Logger.h>
#include <mars_common/exception/SetParamException.h>
#include <mars_common/exception/ReadParamException.h>
#include <mars_routing_core/Step.h>

#include <mars_agent_physical_robot_msgs/MotionAssignment.h>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
namespace behavior
{
/**
 * @brief The PublishMotionAssignmentsToTarget class takes the current route
 * step from the blackboard and publishes two motion
 * assignments to the physical agent based on the step. It also changes the
 * values of the current step and the current sequence number on the blackboard.
 */
class PublishMotionAssignmentsToTarget : public BT::SyncActionNode
{
public:
  /**
   * @brief PublishMotionAssignmentsToTarget Constructs an action behavior tree node.
   * @param pName
   * @param pConfig
   */
  PublishMotionAssignmentsToTarget(const std::string& pName,
                                   const BT::NodeConfiguration& pConfig);

  /**
   * @brief providedPorts
   * @return
   */
  static BT::PortsList providedPorts();

  /**
   * @brief tick Is called by every tree tick.
   * @return
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief getParams Gets necessary parameter from the blackboard
   * @return True, if all parameter could be read.  False, otherwise.
   */
  bool getParams();

  void fillAndPublishToIntersection() noexcept(false);

  void fillAndPublishToTarget() noexcept(false);

  void writeCurrentSequencePosition() noexcept(false);

  void incAndWriteRouteStep(const unsigned int pNumNewAllocatedSteps) noexcept(false);

  const unsigned int readNumNewAllocatedSteps() noexcept(false);

  /**
   * @brief mPathId current path id, gets read from the blackboard
   */
  mars::common::Id mTaskId;

  /**
   * @brief mRouteStep current route step, gets read and write from/on the
   * blackboard.
   */
  mars::routing::core::IterationStep* mRouteStep;

  /**
   * @brief mMotionAssignmentTopic Topic name of the motion assignment
   * publisher, must include the namespace of the physical agent.
   */
  std::string mMotionAssignmentTopic;

  /**
   * @brief mCurrentSequencePosition Holds the current position in the sequence,
   * needed for executing the assignments in the right order (on the physical
   * agent). The blackboard field needs to be set by another action beforehand!
   */
  unsigned int mCurrentSequencePosition;

  /**
   * @brief mMotionPublisher Publishes the constructed motion assignments to the
   * physical agent.
   */
  boost::optional<ros::Publisher> mMotionPublisher;

  /**
   * @brief mStepCount Number of steps in the current route, gets read from the
   * blackboard
   */
   int mStepCount;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // PUBLISHMOTIONASSIGNMENTSTOTARGET_H
