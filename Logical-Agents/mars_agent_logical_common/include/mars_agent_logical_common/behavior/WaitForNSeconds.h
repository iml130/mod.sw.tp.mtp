#ifndef WAITFORNSECONDS_H
#define WAITFORNSECONDS_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/time.h>

#include <mars_common/Logger.h>
#include <mars_common/exception/ReadParamException.h>

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
class WaitForNSeconds : public BT::CoroActionNode
{
public:
  /**
   * @brief WaitForNSeconds Constructs an instance of this class.
   * @param name The name of this node.
   * @param config The configuration of this node. Gets forwarded to the super
   * class.
   */
  WaitForNSeconds(const std::string& name, const BT::NodeConfiguration& config);

  /**
   * @brief tick Implements the logic of this node.
   * @return Succesful, if the task is completed.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief providedPorts Defines the ports of this node.
   * @return List of ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief halt
   */
  void halt() override;

private:
  ros::Duration getWaitDuration();

  void writeFlag(const bool pFlagValue);
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // WAITFORNSECONDS_H
