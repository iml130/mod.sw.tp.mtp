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

#ifndef MARSEDGE_H
#define MARSEDGE_H

#include <mars_common/Id.h>
#include <mars_topology_common/TopologyEntity.h>

#include <vector>

namespace mars
{
namespace topology
{
namespace edge
{

class MarsEdge : public mars::topology::common::TopologyEntity
{
public:
  MarsEdge(const std::string id, const std::string& description, int type, bool isLocked,
           float edgeCoordinateX, float edgeCoordinateY, const std::vector<float>& footprintX,
           const std::vector<float>& footprintY, const std::string& frameId,
           const std::string& originId, const std::string& originContainer,
           const std::string& targetId, const std::string& targetContainer, int pDirection,
           float edgeLength,
           const mars::topology::common::TopologyEntityRestrictions& pRestrictions);

  const mars::common::Id& getOriginId(void) const;
  const mars::common::Id& getTargetId(void) const;
  const mars::common::Id& getOriginContainer(void) const;
  const mars::common::Id& getTargetContainer(void) const;

  float getLength(void) const;
  mars::topology::common::DirectionType getDirection(void) const;

private:
  mars::common::Id mOriginId;
  mars::common::Id mTargetId;
  mars::common::Id mOriginContainer;
  mars::common::Id mTargetContainer;

  float mLength;
  mars::topology::common::DirectionType mDirection;
};

} // namespace edge
} // namespace topology
} // namespace mars

#endif // MARSEDGE_H
