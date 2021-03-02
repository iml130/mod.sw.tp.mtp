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


#include "mars_yellow_pages/MARSYellowPages.h"


MARSYellowPages::MARSYellowPages(){}

MARSYellowPages::~MARSYellowPages(){}

void MARSYellowPages::addContainer(
    const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg)
{
  this->mContainerMap.emplace(
      mars::common::Id(pMsg->container_id).getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC),
      this->createContainerEntitiesSet(pMsg));
}

mars::common::Id MARSYellowPages::getContainerId(
    const mars_topology_srvs::GetTopologyEntityContainerId::Request& pReq)
{
  std::string lTmpEntityIdString =
      mars::common::Id(pReq.entity_id).getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC);
  mars::common::Id lContainerId = mars::common::Id::createInvalidId();

  for (auto& it : this->mContainerMap)
  {
    if (it.second.find(lTmpEntityIdString) != it.second.end())
    {
      lContainerId = mars::common::Id(it.first);
      break;
    }
  }

  return lContainerId;
}

std::set<std::string> MARSYellowPages::createContainerEntitiesSet(
    const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg) const
{
  std::set<std::string> lContainerEntitiesSet;

  for (size_t i = 0; i < pMsg->contained_entity_ids.size(); i++)
  {
    lContainerEntitiesSet.emplace(mars::common::Id(pMsg->contained_entity_ids[i])
                                      .getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC));
  }

  return lContainerEntitiesSet;
}