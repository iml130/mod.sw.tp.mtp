#include "mars_common/geometry/Footprint.h"
#include "mars_common/geometry/Segment.h"
#include "mars_routing_common/geometry/utilities.h"

#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>

TEST(GeometryUtilitiesTests, testSamePoints)
{
  Eigen::Vector3d lOrigin(0, 0, 0);
  Eigen::Vector3d lTarget(0, 0, 0);

  EXPECT_DOUBLE_EQ(0, mars::routing::common::geometry::utilities::get2DAngleBetweenPoints(lOrigin, lTarget));
}

TEST(GeometryUtilitiesTests, footprintSingleIntersection)
{
  mars::common::geometry::Footprint lFootprint = { Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 2),
                                                            Eigen::Vector2d(2, 2), Eigen::Vector2d(2, 0) };

  Eigen::Vector3d lOrigin(-1, 1, 0);
  Eigen::Vector3d lTarget(1, 1, 0);

  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExpectedIntersections{ Eigen::Vector2d(0, 1) };
  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lResultedIntersections(
      mars::routing::common::geometry::utilities::getFootprintIntersections(lFootprint, lOrigin, lTarget));

  EXPECT_EQ(lExpectedIntersections.size(), lResultedIntersections.size());
  EXPECT_EQ(lExpectedIntersections, lResultedIntersections);
}

TEST(GeometryUtilitiesTests, footprintDoubleIntersection)
{
  mars::common::geometry::Footprint lFootprint = { Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 2),
                                                            Eigen::Vector2d(2, 2), Eigen::Vector2d(2, 0) };

  Eigen::Vector3d lOrigin(-1, 1, 0);
  Eigen::Vector3d lTarget(3, 1, 0);

  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExpectedIntersections{ Eigen::Vector2d(0, 1), Eigen::Vector2d(2, 1) };
  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lResultedIntersections(
      mars::routing::common::geometry::utilities::getFootprintIntersections(lFootprint, lOrigin, lTarget));

  EXPECT_EQ(lExpectedIntersections.size(), lResultedIntersections.size());
  EXPECT_EQ(lExpectedIntersections, lResultedIntersections);
}

TEST(GeometryUtilitiesTests, footprintClosingIntersection)
{
  mars::common::geometry::Footprint lFootprint = { Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 2),
                                                            Eigen::Vector2d(2, 2), Eigen::Vector2d(2, 0) };

  Eigen::Vector3d lOrigin(1, -1, 0);
  Eigen::Vector3d lTarget(1, 1, 0);

  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExpectedIntersections{ Eigen::Vector2d(1, 0) };
  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lResultedIntersections(
      mars::routing::common::geometry::utilities::getFootprintIntersections(lFootprint, lOrigin, lTarget));

  EXPECT_EQ(lExpectedIntersections.size(), lResultedIntersections.size());
  EXPECT_EQ(lExpectedIntersections, lResultedIntersections);
}

TEST(GeometryUtilitiesTests, footprintClosingContact)
{
  mars::common::geometry::Footprint lFootprint = { Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 2),
                                                            Eigen::Vector2d(2, 2), Eigen::Vector2d(2, 0) };

  Eigen::Vector3d lOrigin(1, -1, 0);
  Eigen::Vector3d lTarget(1, 0, 0);

  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExpectedIntersections{ Eigen::Vector2d(1, 0) };
  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lResultedIntersections(
      mars::routing::common::geometry::utilities::getFootprintIntersections(lFootprint, lOrigin, lTarget));

  EXPECT_EQ(lExpectedIntersections.size(), lResultedIntersections.size());
  EXPECT_EQ(lExpectedIntersections, lResultedIntersections);
}

TEST(GeometryUtilitiesTests, footprintCornerContact)
{
  mars::common::geometry::Footprint lFootprint = { Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 2),
                                                            Eigen::Vector2d(2, 2), Eigen::Vector2d(2, 0) };

  Eigen::Vector3d lOrigin(1, -1, 0);
  Eigen::Vector3d lTarget(0, 0, 0);

  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExpectedIntersections{ Eigen::Vector2d(0, 0) };
  std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lResultedIntersections(
      mars::routing::common::geometry::utilities::getFootprintIntersections(lFootprint, lOrigin, lTarget));

  EXPECT_EQ(lExpectedIntersections.size(), lResultedIntersections.size());
  EXPECT_EQ(lExpectedIntersections, lResultedIntersections);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}