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

#ifndef ROBOTACTION_H
#define ROBOTACTION_H

#include <mars_agent_physical_robot_msgs/RobotAction.h>

#include <unordered_map>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
class RobotAction
{
public:
  RobotAction(const mars_agent_physical_robot_msgs::RobotAction robotActionMsg);

  bool isManual() const;

  const unsigned int getCategory() const;

  const unsigned int getAction() const;

  const std::vector<mars_common_msgs::Tuple> getAttributes() const;

  const std::string getDescription() const;

private:

  unsigned int mActionCategory;

  unsigned int mAction;

  std::vector<mars_common_msgs::Tuple> mAttributes;

  std::string mDescription;

  std::unordered_map<std::string, unsigned int> mActionEnums;

  void initActionEnum();

};
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars
#endif // ROBOTACTION_H
