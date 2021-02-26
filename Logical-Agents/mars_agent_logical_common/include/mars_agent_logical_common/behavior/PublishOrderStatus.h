#ifndef PUBLISHORDERSTATUS_H
#define PUBLISHORDERSTATUS_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <mars_agent_logical_msgs/OrderStatus.h>
#include <mars_common/Id.h>
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
class PublishOrderStatus : public BT::SyncActionNode
{
public:
  PublishOrderStatus(const std::string& pName,
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

  ros::Publisher mOrderStatusPub;

  unsigned int mCurrentOrderStatus;
  unsigned int mCurrentOrderType;
  unsigned int mCurrentOrderState;
  mars::common::Id mCurrentOrderId;

};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // PUBLISHORDERSTATUS_H
