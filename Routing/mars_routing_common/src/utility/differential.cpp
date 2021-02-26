#include "mars_agent_physical_common/RobotAgentProperties.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/ConstantProfile.h"
#include "mars_routing_common/utility/MotionProfile.h"
#include "mars_routing_common/utility/differential.h"

template <class Profile>
void mars::routing::common::utility::
    differential<mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge, Profile>::setMotion(
        boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
        boost::optional<double>& pExitMotion, double pRotationAngle,
        mars::routing::common::topology::Vertex& pCurrentEntity, mars::routing::common::topology::Edge& pNextEntity)
{
  pEntryMotion = boost::none;
  pAngularMotion = abs(pRotationAngle);

  boost::optional<Eigen::Vector3d> lCurrentLocation = pCurrentEntity.getLocation();
  boost::optional<Eigen::Vector3d> lNextLocation = pNextEntity.getLocation();
  boost::optional<mars::common::geometry::Footprint&> lCurrentFootprint = pCurrentEntity.getFootprint();

  if (!lCurrentLocation || !lCurrentFootprint)
  {
    pExitMotion = boost::none;
    return;
  }

  if (!lNextLocation)
  {
    pExitMotion = boost::none;
  }
  else
  {
    std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExitIntersections =
        mars::routing::common::geometry::utilities::getFootprintIntersections(*lCurrentFootprint, *lCurrentLocation,
                                                                              *lNextLocation);

    if (lExitIntersections.size() == 1)
    {
      pExitMotion = (lCurrentLocation->block(0, 0, 2, 1) - *lExitIntersections.begin()).norm();
    }
    else
    {
      pExitMotion = boost::none;
    }
  }
};

template <class Profile>
void mars::routing::common::utility::
    differential<mars::routing::common::topology::Vertex, mars::routing::common::topology::Edge, Profile>::setMotion(
        boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
        boost::optional<double>& pExitMotion, double pRotationAngle,
        mars::routing::common::topology::Vertex& pCurrentEntity, mars::routing::common::topology::Edge& pNextEntity,
        mars::routing::common::topology::Edge& pPreviousEntity)
{
  pAngularMotion = abs(pRotationAngle);

  boost::optional<Eigen::Vector3d> lCurrentLocation = pCurrentEntity.getLocation();
  boost::optional<Eigen::Vector3d> lNextLocation = pNextEntity.getLocation();
  boost::optional<Eigen::Vector3d> lPreviousLocation = pPreviousEntity.getLocation();
  boost::optional<mars::common::geometry::Footprint&> lCurrentFootprint = pCurrentEntity.getFootprint();

  if (!lCurrentLocation || !lCurrentFootprint)
  {
    pEntryMotion = boost::none;
    pExitMotion = boost::none;
    return;
  }

  if (!lNextLocation)
  {
    pExitMotion = boost::none;
  }
  else
  {
    std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExitIntersections =
        mars::routing::common::geometry::utilities::getFootprintIntersections(*lCurrentFootprint, *lCurrentLocation,
                                                                              *lNextLocation);

    if (lExitIntersections.size() == 1)
    {
      pExitMotion = (lCurrentLocation->block(0, 0, 2, 1) - *lExitIntersections.begin()).norm();
    }
    else
    {
      pExitMotion = boost::none;
    }
  }

  if (!lPreviousLocation)
  {
    pEntryMotion = boost::none;
  }
  else
  {
    std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lEntryIntersections =
        mars::routing::common::geometry::utilities::getFootprintIntersections(*lCurrentFootprint, *lPreviousLocation,
                                                                              *lCurrentLocation);

    if (lEntryIntersections.size() == 1)
    {
      pEntryMotion = (lCurrentLocation->block(0, 0, 2, 1) - *lEntryIntersections.begin()).norm();
    }
    else
    {
      pEntryMotion = 0;
    }
  }
};

template <class Profile>
void mars::routing::common::utility::
    differential<mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex, Profile>::setMotion(
        boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
        boost::optional<double>& pExitMotion, double pRotationAngle,
        mars::routing::common::topology::Edge& pCurrentEntity, mars::routing::common::topology::Vertex& pNextEntity)
{
  pEntryMotion = boost::none;
  pAngularMotion = abs(pRotationAngle);

  boost::optional<Eigen::Vector3d> lCurrentLocation = pCurrentEntity.getLocation();
  boost::optional<Eigen::Vector3d> lNextLocation = pNextEntity.getLocation();
  boost::optional<mars::common::geometry::Footprint&> lTargetFootprint = pNextEntity.getFootprint();

  if (!lCurrentLocation || !lTargetFootprint)
  {
    pExitMotion = boost::none;
    return;
  }

  if (!lNextLocation)
  {
    pExitMotion = boost::none;
  }
  else
  {
    std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExitIntersections =
        mars::routing::common::geometry::utilities::getFootprintIntersections(*lTargetFootprint, *lCurrentLocation,
                                                                              *lNextLocation);

    if (lExitIntersections.size() == 1)
    {
      pExitMotion = (lCurrentLocation->block(0, 0, 2, 1) - *lExitIntersections.begin()).norm();
    }
    else
    {
      pExitMotion = boost::none;
    }
  }
};

template <class Profile>
void mars::routing::common::utility::
    differential<mars::routing::common::topology::Edge, mars::routing::common::topology::Vertex, Profile>::setMotion(
        boost::optional<double>& pAngularMotion, boost::optional<double>& pEntryMotion,
        boost::optional<double>& pExitMotion, double pRotationAngle,
        mars::routing::common::topology::Edge& pCurrentEntity, mars::routing::common::topology::Vertex& pNextEntity,
        mars::routing::common::topology::Vertex& pPreviousEntity)
{
  pAngularMotion = abs(pRotationAngle);

  boost::optional<Eigen::Vector3d> lCurrentLocation = pCurrentEntity.getLocation();
  boost::optional<Eigen::Vector3d> lNextLocation = pNextEntity.getLocation();
  boost::optional<Eigen::Vector3d> lPreviousLocation = pPreviousEntity.getLocation();
  boost::optional<mars::common::geometry::Footprint&> lTargetFootprint = pNextEntity.getFootprint();
  boost::optional<mars::common::geometry::Footprint&> lPreviousFootprint = pPreviousEntity.getFootprint();

  if (!lCurrentLocation || !lTargetFootprint)
  {
    pEntryMotion = boost::none;
    pExitMotion = boost::none;
    return;
  }

  if (!lNextLocation)
  {
    pExitMotion = boost::none;
  }
  else
  {
    std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lExitIntersections =
        mars::routing::common::geometry::utilities::getFootprintIntersections(*lTargetFootprint, *lCurrentLocation,
                                                                              *lNextLocation);

    if (lExitIntersections.size() == 1)
    {
      pExitMotion = (lCurrentLocation->block(0, 0, 2, 1) - *lExitIntersections.begin()).norm();
    }
    else
    {
      pExitMotion = boost::none;
    }
  }

  if (!lPreviousLocation)
  {
    pEntryMotion = boost::none;
  }
  else
  {
    std::set<Eigen::Vector2d, mars::routing::common::geometry::EigenMatrixCompare> lEntryIntersections =
        mars::routing::common::geometry::utilities::getFootprintIntersections(*lPreviousFootprint, *lPreviousLocation,
                                                                              *lCurrentLocation);

    if (lEntryIntersections.size() == 1)
    {
      pEntryMotion = (lCurrentLocation->block(0, 0, 2, 1) - *lEntryIntersections.begin()).norm();
    }
    else
    {
      pEntryMotion = 0;
    }
  }
};

template class mars::routing::common::utility::differential<mars::routing::common::topology::Vertex,
                                                              mars::routing::common::topology::Edge,
                                                              mars::routing::common::utility::AffineProfile>;

template class mars::routing::common::utility::differential<mars::routing::common::topology::Edge,
                                                              mars::routing::common::topology::Vertex,
                                                              mars::routing::common::utility::AffineProfile>;
