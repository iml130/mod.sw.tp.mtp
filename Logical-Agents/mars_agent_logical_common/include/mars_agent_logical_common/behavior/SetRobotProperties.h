#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETROBOTPROPERTIES_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETROBOTPROPERTIES_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <mutex>
#include <ros/ros.h>

#include <boost/optional.hpp>

#include <eigen3/Eigen/Geometry>

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_agent_physical_robot_msgs/RobotAgentProperties.h"

static const std::string BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_ROBOT_PROPERTIES = "robot_properties";
static const std::string BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TOPIC_NAME_ROBOT_PROPERTIES =
    "topic_name_robot_properties";

static const std::string BEHAVIOR_SETCURRENTMOTION_DEFAULT_TOPIC_NAME_ROBOT_PROPERTIES =
    "robot_description";

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
class SetRobotProperties : public BT::CoroActionNode
{
public:
  SetRobotProperties(const std::string& pName, const BT::NodeConfiguration& pConfig)
      : BT::CoroActionNode(pName, pConfig)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(
                BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TOPIC_NAME_ROBOT_PROPERTIES),
            BT::OutputPort<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
                BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_ROBOT_PROPERTIES)};
  }

  BT::NodeStatus tick() override;
  void halt() override{};

private: // functions
  void subscriberCallbackRobotProperties(
      const mars_agent_physical_robot_msgs::RobotAgentProperties::ConstPtr& pRobotProperties);

private: // variables
  ros::NodeHandle mNodeHandle;

  boost::optional<mars::agent::physical::common::RobotAgentProperties> mRobotProperties;
  boost::optional<ros::Subscriber> mRobotPropertiesSubscriber;

  mutable std::mutex mRobotPropertiesMutex;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETROBOTPROPERTIES_H
