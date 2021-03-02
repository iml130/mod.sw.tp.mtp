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

#ifndef REPEATUNTILDONE_H
#define REPEATUNTILDONE_H

#include <behaviortree_cpp_v3/decorator_node.h>
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

class RepeatUntilDone : public BT::DecoratorNode
{
public:
  RepeatUntilDone(const std::string& pName);

  RepeatUntilDone(const std::string& name, const BT::NodeConfiguration& config);

  virtual ~RepeatUntilDone() override = default;

  /**
   * @brief providedPorts
   * @return
   */
  static BT::PortsList providedPorts();

private:
  virtual BT::NodeStatus tick() override;

  const bool readDoneStatus() noexcept(false);

  void halt() override;
};

} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // REPEATUNTILDONE_H
