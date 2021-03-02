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

#ifndef MARS_ROUTING_PLUGIN_CARP_UTILITIES_H
#define MARS_ROUTING_PLUGIN_CARP_UTILITIES_H

#include <boost/optional.hpp>

#include "mars_common/TimeInterval.h"

namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
class utilities // static
{
public:
  static boost::optional<ros::Time>
  getReservationStartTime(const mars::common::TimeInterval& pTargetFreeTimeInterval,
                          const ros::Time& lTargetEntryTime,
                          const ros::Duration& pLeastTargetOccupationDuration);

private:
  utilities();

  static ros::Duration
  calcLeastReservationDuration(const mars::common::TimeInterval& pTargetFreeTimeInterval,
                               const ros::Time& pTargetEntryTime,
                               const ros::Duration& pLeastTargetOccupationDuration);
};
} // namespace carp
} // namespace plugin
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_PLUGIN_CARP_UTILITIES_H
