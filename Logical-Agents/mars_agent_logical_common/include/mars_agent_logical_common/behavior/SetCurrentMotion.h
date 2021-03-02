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

#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTMOTION_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTMOTION_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <mutex>
#include <ros/ros.h>

#include <boost/optional.hpp>
#include <eigen3/Eigen/Geometry>
#include <list>

#include "mars_agent_physical_robot_msgs/Motion.h"

static const std::string BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_POSE = "pose";
static const std::string BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TWIST = "twist";
static const std::string BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TOPIC_NAME_MOTION =
    "topic_name_motion";

static const std::string BEHAVIOR_SETCURRENTMOTION_DEFAULT_TOPIC_NAME_MOTION = "current_motion";

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
class SetCurrentMotion : public BT::CoroActionNode
{
public:
  SetCurrentMotion(const std::string& pName, const BT::NodeConfiguration& pConfig);

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>(BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TOPIC_NAME_MOTION),
        BT::OutputPort<Eigen::Affine3d>(BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_POSE),
        BT::OutputPort<Eigen::Matrix<double, 6, 1>>(BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TWIST)};
  }

  BT::NodeStatus tick() override;
  void halt() override { CoroActionNode::halt(); };

private: // functions
  void subscriberCallbackCurrentMotion(
      const mars_agent_physical_robot_msgs::Motion::ConstPtr& pMotionMsg);

private: // variables
  ros::NodeHandle mNodeHandle;

  boost::optional<ros::Subscriber> mMotionSubscriber;

  boost::optional<Eigen::Affine3d> mPose;
  boost::optional<Eigen::Matrix<double, 6, 1>> mTwist;

  std::list<mars_agent_physical_robot_msgs::Motion> mCurrentMotionBuffer;

  mutable std::mutex mMotionMutex;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_SETCURRENTMOTION_H
