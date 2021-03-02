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

#ifndef MARS_ROUTING_COMMON_UTILITY_DIFFERENTIAL_H
#define MARS_ROUTING_COMMON_UTILITY_DIFFERENTIAL_H

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_common/topology/Vertex.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/ConstantProfile.h"

#include "mars_common/geometry/Segment.h"
#include "mars_routing_common/geometry/utilities.h"

#include <stdlib.h>
#include <algorithm>
#include <type_traits>

#include <boost/geometry/algorithms/intersection.hpp>
#include <eigen3/Eigen/Core>

#include <ros/time.h>

#include <boost/optional.hpp>

namespace mars
{
namespace routing
{
namespace common
{
namespace utility
{
template <class Origin, class Target, class Profile>
class differential
{
public:
  /**
   * @brief setMotion Calculates angular and linear motions inside a topology entity.
   * @param pAngularMotion Angular motion to calculate. Sets reference inside this function.
   * @param pEntryMotion Linear entry motion to calculate. Sets reference inside this function.
   * @param pExitMotion Linear exit motion to calculate. Sets reference inside this function.
   * @param pRotationAngle Required angle to rotate on the resource.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   */
  static void setMotion(boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
                        boost::optional<double>& pExitMotion, double pRotationAngle, Origin& pCurrentEntity,
                        Target& pNextEntity);

  /**
   * @brief setMotion Calculates angular and linear motions inside a topology entity.
   * @param pAngularMotion Angular motion to calculate. Sets reference inside this function.
   * @param pEntryMotion Linear entry motion to calculate. Sets reference inside this function.
   * @param pExitMotion Linear exit motion to calculate. Sets reference inside this function.
   * @param pRotationAngle Required angle to rotate on the resource.
   * @param pCurrentEntity Resource to calculate the occupation time for.
   * @param pNextEntity Successive topology entity on the routing path.
   * @param pPreviousEntity Previous topology entity on the routing path.
   */
  static void setMotion(boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
                        boost::optional<double>& pExitMotion, double pRotationAngle, Origin& pCurrentEntity,
                        Target& pNextEntity, Target& pPreviousEntity);
};

template <class Profile>
class differential<mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge, Profile>
{
public:
  static void setMotion(boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
                        boost::optional<double>& pExitMotion, double pRotationAngle,
                        mars::routing::common::topology::Vertex& pCurrentEntity,
                        mars::routing::common::topology::Edge& pNextEntity);

  static void setMotion(boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
                        boost::optional<double>& pExitMotion, double pRotationAngle,
                        mars::routing::common::topology::Vertex& pCurrentEntity,
                        mars::routing::common::topology::Edge& pNextEntity,
                        mars::routing::common::topology::Edge& pPreviousEntity);
};

template <class Profile>
class differential<mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex, Profile>
{
public:
  static void setMotion(boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
                        boost::optional<double>& pExitMotion, double pRotationAngle,
                        mars::routing::common::topology::Edge& pCurrentEntity,
                        mars::routing::common::topology::Vertex& pNextEntity);

  static void setMotion(boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
                        boost::optional<double>& pExitMotion, double pRotationAngle,
                        mars::routing::common::topology::Edge& pCurrentEntity,
                        mars::routing::common::topology::Vertex& pNextEntity,
                        mars::routing::common::topology::Vertex& pPreviousEntity);
};
}  // namespace utility
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_UTILITY_TIMEPREDICTION_H