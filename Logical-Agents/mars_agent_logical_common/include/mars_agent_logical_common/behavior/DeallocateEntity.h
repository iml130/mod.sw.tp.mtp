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

#ifndef MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_DEALLOCATEENTITY_H
#define MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_DEALLOCATEENTITY_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <eigen3/Eigen/Geometry>

#include <tf2_ros/transform_listener.h>

#include <mars_agent_physical_common/RobotAgentProperties.h>

#include <mars_common/behavior/AsyncCoroActionNode.h>

#include <mars_routing_core/Step.h>

#include <mars_routing_common/topology/Edge.h>
#include <mars_routing_common/topology/Vertex.h>



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
class DeallocateEntity : public BT::CoroActionNode
{
public:
  DeallocateEntity(const std::string& pName,
                   const BT::NodeConfiguration& pConfig);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void halt() override;

private:
  bool getParams();

  bool setParams();

  /**
   * @brief getRobotCoordinate Extracts the 2D coordinate from the Affine3d pose.
   * @return 2D coordinate of the robot.
   */
  Eigen::Vector2d getRobotCoordinate();

  /**
   * @brief updateRobotPose Reads the current robot pose from the blackboard and stores it.
   * @return True, if value could be read from the blackboard.
   */
  bool updateRobotPose();

  bool updateAllocationCount();

  boost::optional<mars::common::Id> mAgentId;
  boost::optional<mars::common::Id> mRouteId;
  boost::optional<std::string> mPhysicalAgentNamespace;

  std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>
      mRobotProperties;

  Eigen::Affine3d mRobotPose;

  unsigned int mAllocationCount;
  mars::routing::core::IterationStep* mDeallocateStep;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_BEHAVIOR_DEALLOCATEENTITY_H
