#ifndef WAITFORMANUALTRIGGER_H
#define WAITFORMANUALTRIGGER_H

#include <atomic>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>

#include <mars_agent_logical_srvs/ManualActionDone.h>
#include <mars_agent_physical_robot_msgs/RobotAction.h>
#include <mars_common/Logger.h>

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
class WaitForManualTrigger : public BT::CoroActionNode
{
public:
  /**
   * @brief WaitForManualTrigger Constructs an instance of this class.
   * @param name The name of this node.
   * @param config The configuration of this node. Gets forwarded to the super class.
   */
  WaitForManualTrigger(const std::string& name,
                       const BT::NodeConfiguration& config);

  /**
   * @brief tick Implements the logic of this node.
   * @return Succesful, if the task is completed.
   */
  BT::NodeStatus tick() override;
  void halt() override { CoroActionNode::halt(); };

  /**
   * @brief providedPorts Defines the ports of this node.
   * @return List of ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief serviceTriggerCallback Gets called when a client calls the manual trigger service.
   * @param pReq Request of the defined service
   * @param pRes Response of the defined service
   * @return True, if the service call is processed cleanly.
   */
  static bool serviceTriggerCallback(
      mars_agent_logical_srvs::ManualActionDoneRequest& pReq,
      mars_agent_logical_srvs::ManualActionDoneResponse& pRes);

private:
  /**
   * @brief mWaitForTrigger Flag to indicate, that a tigger event is expected.
   */
  static std::atomic<bool> mWaitForTrigger;

  /**
   * @brief mTriggerReceived Flag to indicate, that a trigger was received.
   */
  static std::atomic<bool> mTriggerReceived;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // WAITFORMANUALTRIGGER_H
