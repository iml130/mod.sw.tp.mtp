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

#ifndef REPEATUNTILSUCCESSFUL_H
#define REPEATUNTILSUCCESSFUL_H

#include <behaviortree_cpp_v3/decorator_node.h>

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
class RepeatUntilSuccessful : public BT::DecoratorNode
{
public:
  RepeatUntilSuccessful(const std::string& pName);

  virtual ~RepeatUntilSuccessful() override = default;

private:
  virtual BT::NodeStatus tick() override;

  void halt() override;
};

} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // REPEATUNTILSUCCESFUL_H
