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

#ifndef ALLOCATEENTITY_H
#define ALLOCATEENTITY_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <mars_common/Id.h>
#include <mars_common/Logger.h>
#include <mars_common/behavior/Utils.hpp>
#include <mars_common/exception/ReadParamException.h>
#include <mars_common/exception/SetParamException.h>
#include <mars_routing_core/Step.h>

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
 * @brief The AllocateEntity class
 */
class AllocateEntity : public BT::CoroActionNode
{
public:
  /**
   * @brief AllocateEntity
   * @param pName
   * @param pConfig
   */
  AllocateEntity(const std::string& pName, const BT::NodeConfiguration& pConfig);

  /**
   * @brief providedPorts
   * @return
   */
  static BT::PortsList providedPorts();

  /**
   * @brief tick
   * @return
   */
  BT::NodeStatus tick() override;

  /**
   * @brief halt
   */
  void halt() override;

private:
  /**
   * @brief getParams
   * @return
   */
  bool getParams();

  unsigned int readAllocationCount() noexcept(false);

  void waitForNFreeAllocations(const unsigned int pWantedAllocations);

  bool fitsOnCurrentTarget( mars::routing::common::topology::Entity& pEntity,
                           mars::agent::physical::common::RobotAgentProperties pRAP);

  void incAndWriteAllocationCount(const unsigned int pNumOfNewAllocations) noexcept(false);

  void writeNumNewAllocatedCount(const unsigned int pNumOfNewAllocations) noexcept(false);

  void setDoneOnBlackboard() noexcept(false);

  bool allocate(mars::routing::core::IterationStep* pRouteStep);

  mars::common::Id mPathId;

  mars::routing::core::IterationStep* mRouteStep;
  unsigned int mAllocationCount;
  boost::optional<mars::common::Id> mAgentId;
  boost::optional<unsigned int> mAllocationLimit;
  boost::optional<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>
      mRobotProperties;
};
} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // ALLOCATEENTITY_H
