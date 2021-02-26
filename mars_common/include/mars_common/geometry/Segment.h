#ifndef MARS_COMMON_GEOMETRY_SEGMENT_H
#define MARS_COMMON_GEOMETRY_SEGMENT_H

#include "mars_common/geometry/Point.h"

#include <boost/geometry.hpp>
#include <eigen3/Eigen/Core>

namespace mars
{
namespace common
{
namespace geometry
{
typedef boost::geometry::model::segment<Eigen::Vector2d> Segment;
} // namespace geometry
} // namespace common
} // namespace mars

#endif // MARS_COMMON_GEOMETRY_SEGMENT_H