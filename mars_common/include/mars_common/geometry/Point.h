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

#ifndef MARS_COMMON_GEOMETRY_POINT_H
#define MARS_COMMON_GEOMETRY_POINT_H

#include <boost/geometry.hpp>
#include <eigen3/Eigen/Core>

#include <cstddef>

namespace boost
{
namespace geometry
{
namespace traits
{
template <typename ScalarType, int nDimensions>
struct tag<Eigen::Matrix<ScalarType, nDimensions, 1>>
{
  typedef point_tag type;
};

template <typename ScalarType, int nDimensions>
struct coordinate_type<Eigen::Matrix<ScalarType, nDimensions, 1>>
{
  typedef ScalarType type;
};

template <typename ScalarType, int nDimensions>
struct coordinate_system<Eigen::Matrix<ScalarType, nDimensions, 1>>
{
  typedef cs::cartesian type;
};

template <typename ScalarType, int nDimensions>
struct dimension<Eigen::Matrix<ScalarType, nDimensions, 1>> : boost::mpl::int_<nDimensions>
{
};

template <typename ScalarType, int nDimensions, std::size_t dimension>
struct access<Eigen::Matrix<ScalarType, nDimensions, 1>, dimension>
{
  static inline ScalarType get(Eigen::Matrix<ScalarType, nDimensions, 1> const& vector)
  {
    return vector(dimension);
  }

  static inline void set(Eigen::Matrix<ScalarType, nDimensions, 1>& vector, ScalarType const& value)
  {
    vector(dimension) = value;
  }
};

} // namespace traits
} // namespace geometry
} // namespace boost

#endif // MARS_COMMON_GEOMETRY_POINT_H