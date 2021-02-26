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