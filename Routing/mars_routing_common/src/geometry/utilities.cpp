#include "mars_routing_common/geometry/utilities.h"

#include "mars_common/geometry/Segment.h"

#include <cmath>
#include <eigen3/Eigen/Core>

double mars::routing::common::geometry::utilities::get2DAngleBetweenPoints(const Eigen::Vector3d& pOrigin,
                                                                           const Eigen::Vector3d& pTarget)
{
  double angle = std::atan2((pTarget.y() - pOrigin.y()), (pTarget.x() - pOrigin.x()));

  if (angle < 0)
  {
    angle += 2 * M_PI;
  }

  return angle;
}

double mars::routing::common::geometry::utilities::get2DRotationAngle(const Eigen::Vector3d& pOrigin,
                                                                      const Eigen::Vector3d& pTarget,
                                                                      const double& pOrientation)
{
  double angleHop = get2DAngleBetweenPoints(pOrigin, pTarget);
  double angleAbsDiff = abs(pOrientation - angleHop);
  double angle = (angleAbsDiff > M_PI) ? (2 * M_PI - angleAbsDiff) : angleAbsDiff;

  return angle;
}

std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare>
mars::routing::common::geometry::utilities::getFootprintIntersections(
    const mars::common::geometry::Footprint pFootprint, const Eigen::Vector2d& pOrigin,
    const Eigen::Vector2d& pTarget)
{
  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lFootprintIntersections;

  mars::common::geometry::Segment lSegment(pOrigin, pTarget);

  for (std::size_t i = 0; i < pFootprint.size(); i++)
  {
    Eigen::Vector2d lOrigin = pFootprint[i];
    Eigen::Vector2d lTarget;

    if (i + 1 < pFootprint.size())
    {
      lTarget = pFootprint[i + 1];
    }
    else
    {
      lTarget = pFootprint[0];
    }

    mars::common::geometry::Segment lFootprintSegment(lOrigin, lTarget);
    std::vector<Eigen::Vector2d> lIntersections;

    if (boost::geometry::intersection(lFootprintSegment, lSegment, lIntersections))
    {
      for (auto& iIntersection : lIntersections)
      {
        lFootprintIntersections.insert(iIntersection);
      }
    }
  }

  return lFootprintIntersections;
}

std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare>
mars::routing::common::geometry::utilities::getFootprintIntersections(
    const mars::common::geometry::Footprint pFootprint, const Eigen::Vector3d& pOrigin,
    const Eigen::Vector3d& pTarget)
{
  Eigen::Vector2d lOrigin = pOrigin.block(0, 0, 2, 1);
  Eigen::Vector2d lTarget = pTarget.block(0, 0, 2, 1);

  return mars::routing::common::geometry::utilities::getFootprintIntersections(pFootprint, lOrigin, lTarget);
}