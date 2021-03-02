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

#ifndef MARS_ROUTING_COMMON_TOPOLOGY_OCCUPATIONDURATIONHASHKEY_H
#define MARS_ROUTING_COMMON_TOPOLOGY_OCCUPATIONDURATIONHASHKEY_H

#include "mars_agent_physical_common/RobotAgentProperties.h"

#include "mars_routing_common/topology/Entity.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
template <class Target>
class OccupationDurationKey
{
public:
  OccupationDurationKey(const Target& pOrigin, const Target& pTarget,
                        const mars::agent::physical::common::RobotAgentProperties& pRAP, const double& pVelocity);

  OccupationDurationKey(const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey);

  ~OccupationDurationKey();

  OccupationDurationKey&
  operator=(const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey) = delete;

  const Target* getOrigin() const;
  const Target* getTarget() const;
  const mars::agent::physical::common::RobotAgentProperties& getRobotAgentProperties() const;
  const double& getVelocity() const;

  bool operator==(const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey) const;

  struct Hash
  {
    std::size_t operator()(const mars::routing::common::topology::OccupationDurationKey<Target>& pKey) const;
  };

private:
  const Target* mOrigin;
  const Target* mTarget;
  const mars::agent::physical::common::RobotAgentProperties mRAP;
  const double mVelocity;
};
}  // namespace topology
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_TOPOLOGY_OCCUPATIONDURATIONHASHKEY_H