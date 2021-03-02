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

#ifndef MARS_ROUTING_COMMON_TOPOLOGY_SHORTESTOCCUPATIONDURATIONHASHKEY_H
#define MARS_ROUTING_COMMON_TOPOLOGY_SHORTESTOCCUPATIONDURATIONHASHKEY_H

#include "mars_agent_physical_common/RobotAgentProperties.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
template <class Origin>
class ShortestOccupationDurationKey
{
public:
  ShortestOccupationDurationKey(const Origin& pOrigin, const mars::agent::physical::common::RobotAgentProperties& pRAP,
                                const double& pVelocity);

  ShortestOccupationDurationKey(
      const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey);

  ~ShortestOccupationDurationKey();

  ShortestOccupationDurationKey&
  operator=(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey) = delete;

  const Origin* getOrigin() const;
  const mars::agent::physical::common::RobotAgentProperties& getRobotAgentProperties() const;
  const double& getVelocity() const;

  bool operator==(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey) const;

  struct Hash
  {
    std::size_t operator()(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pKey) const;
  };

private:
  const Origin* mOrigin;
  const mars::agent::physical::common::RobotAgentProperties mRAP;
  const double mVelocity;
};
}  // namespace topology
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_TOPOLOGY_SHORTESTOCCUPATIONDURATIONHASHKEY_H