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

#ifndef MARS_ROUTING_PLUGIN_CARP_PLANNINGROUTE_H
#define MARS_ROUTING_PLUGIN_CARP_PLANNINGROUTE_H

#include "mars_common/Id.h"

namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
/**
 * @class PlanningRoute
 */
class PlanningRoute
{
    public:
        virtual bool reserve(const mars::common::Id& pAgentId) = 0;
    protected:
    private:
};
}  // namespace carp
}  // namespace plugin
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_PLUGIN_CARP_PLANNINGROUTE_H