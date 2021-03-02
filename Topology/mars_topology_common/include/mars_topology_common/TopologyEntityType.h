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


#ifndef MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYTYPE_H
#define MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYTYPE_H

#include <mars_common/exception/SetParamException.h>

#include <string>

namespace mars
{
namespace topology
{
namespace common
{

enum class DirectionType : int
{
  unidirectional = 0,
  unidirectional_reversed = 1,
  bidirectional = 2
};

class TopologyEntityType
{
public:

  /**
   * @brief Construct a new Topology Entity Type object
   * 
   * @param pType 
   * @throw mars::common::exception::SetParamException
   */
  TopologyEntityType(int pType) noexcept(false);

  int getType(void) const;

  bool operator==(TopologyEntityType otherType);

private:
  int mType;

  /**
   * @brief Set the Type object
   * 
   * @param pType 
   * @throw mars::common::exception::SetParamException
   */
  void setType(int pType) noexcept(false);
};
}
}
}

namespace std
{
string to_string(const mars::topology::common::TopologyEntityType& pTopologyEntityType);
}

#endif // MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYTYPE_H
