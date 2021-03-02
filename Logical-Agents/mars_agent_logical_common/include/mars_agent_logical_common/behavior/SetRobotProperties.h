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
