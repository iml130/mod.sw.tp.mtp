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