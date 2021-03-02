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


#ifndef MARS_YELLOW_PAGES_H
#define MARS_YELLOW_PAGES_H

#include <mars_common/Id.h>
#include <mars_topology_msgs/TopologyEntityRegistration.h>
#include <mars_topology_srvs/GetTopologyEntityContainerId.h>

#include <map>
#include <set>
#include <string>

class MARSYellowPages
{
public:
  MARSYellowPages();
  ~MARSYellowPages();
  void addContainer(const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg);

  mars::common::Id
  getContainerId(const mars_topology_srvs::GetTopologyEntityContainerId::Request& pReq);

private:
  std::map<std::string, std::set<std::string>> mContainerMap;

  std::set<std::string> createContainerEntitiesSet(
      const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg) const;
};

#endif // MARS_YELLOW_PAGES_H