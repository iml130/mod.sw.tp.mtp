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

#ifndef GEOMETRY_UTILITIES_H
#define GEOMETRY_UTILITIES_H

#include <boost/geometry.hpp>
#include <eigen3/Eigen/Core>

#include "mars_common/geometry/Footprint.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace geometry
{
struct EigenMatrixCompare
{
  template <typename ScalarType, int nRows, int nColumns>
  bool operator()(const Eigen::Matrix<ScalarType, nRows, nColumns>& lhs, const Eigen::Matrix<ScalarType, nRows, nColumns>& rhs)
  {

    return lhs.size() < rhs.size();
  }
};

class utilities  // static
{
public:
  /**
   * @brief get2DRotationAngle Calculates angle between two Point on the XY-plane.
   * @param pOrigin Point of origin.
   * @param pTarget Targeted point.
   * @return Angle in degrees between this and the other point periodically normed to 180°. Can be
   * negative based on the turning direction.
   */
  static double get2DAngleBetweenPoints(const Eigen::Vector3d& pOrigin, const Eigen::Vector3d& pTarget);

  /**
   * @brief get2DRotationAngle Calculates the rotation angle between two 2D points, given a current
   * orientation of an agent.
   * @param pOrigin Point of origin.
   * @param pTarget Targeted point.
   * @param pOrientation Current orientation of the agent.
   * @return Angle in degrees it would take to make a robot agent with the given orientation on the
   * origin to turn towards the target.
   */
  static double get2DRotationAngle(const Eigen::Vector3d& pOrigin, const Eigen::Vector3d& pTarget,
                                   const double& pOrientation);

  static std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare>
  getFootprintIntersections(const mars::common::geometry::Footprint pFootprint, const Eigen::Vector2d& pOrigin,
                            const Eigen::Vector2d& pTarget);

  static std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare>
  getFootprintIntersections(const mars::common::geometry::Footprint pFootprint, const Eigen::Vector3d& pOrigin,
                            const Eigen::Vector3d& pTarget);

private:
  utilities();
};
}  // namespace geometry
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // GEOMETRY_UTILITIES_H