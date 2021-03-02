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