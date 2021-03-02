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

#ifndef MARS_COMMON_GEOMETRY_FOOTPRINT_H
#define MARS_COMMON_GEOMETRY_FOOTPRINT_H

#include "mars_common/geometry/Point.h"

#include <boost/geometry.hpp>
#include <eigen3/Eigen/Core>
#include <vector>

namespace boost
{
namespace geometry
{
namespace traits
{
template <typename Point> struct tag<std::vector<Point>>
{
  typedef ring_tag type;
};

template <typename Point> struct point_order<std::vector<Point>>
{
  static const order_selector value = clockwise;
};

template <typename Point> struct closure<std::vector<Point>>
{
  static const closure_selector value = open;
};
} // namespace traits
} // namespace geometry
} // namespace boost

namespace mars
{
namespace common
{
namespace geometry
{
typedef std::vector<Eigen::Vector2d> Footprint;
} // namespace geometry
} // namespace common
} // namespace mars

#endif // MARS_COMMON_GEOMETRY_FOOTPRINT_H