/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : TODO: INSERT PROJECT NAME
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Marc Down, Max Mustermann
 * Contact(s)       : marc.down@iml.fraunhofer.de
 *                    max.mustermann@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#include "mars_topology_vertex/MarsVertex.h"

#include <ros/console.h>

static const std::string EXCEPTION_MSG_INVALID_TIME_INTERVAL = "Invalid time interval. Interval "
                                                               "overlaps with others!";

mars::topology::vertex::MarsVertex::MarsVertex(
    const std::string id, const std::string& description, int type, bool isLocked,
    float vertexCoordinateX, float vertexCoordinateY, const std::vector<float>& footprintX,
    const std::vector<float>& footprintY, const std::string& frameId,
    const std::vector<mars::common::Id>& outgoingEdgeIds,
    const std::vector<std::string>& outgoingContainers,
    const std::vector<mars::common::Id>& ingoingEdgeIds,
    const std::vector<std::string>& ingoingContainers,
    const mars::topology::common::TopologyEntityRestrictions& pRestrictions)
    : mars::topology::common::TopologyEntity(id, description, type, isLocked, vertexCoordinateX,
                                             vertexCoordinateY, footprintX, footprintY, frameId,
                                             pRestrictions)
{
  this->mOutgoingEdgeIds = outgoingEdgeIds;
  this->mIngoingEdgeIds = ingoingEdgeIds;
  this->mOutgoingContainerIds = setIds(outgoingContainers);
  this->mIngoingContainerIds = setIds(ingoingContainers);
}

mars::topology::vertex::MarsVertex::~MarsVertex() {}

std::vector<mars::common::Id> mars::topology::vertex::MarsVertex::getIngoingEdgeIds() const
{
  return this->mIngoingEdgeIds;
}

std::vector<mars::common::Id> mars::topology::vertex::MarsVertex::getOutgoingEdgeIds() const
{
  return this->mOutgoingEdgeIds;
}

std::vector<mars::common::Id> mars::topology::vertex::MarsVertex::getOutgoingContainerIds() const
{
  return this->mOutgoingContainerIds;
}

std::vector<mars::common::Id> mars::topology::vertex::MarsVertex::getIngoingContainerIds() const
{
  return this->mIngoingContainerIds;
}

std::vector<mars::common::Id> mars::topology::vertex::MarsVertex::setIds(std::vector<std::string> pIds) const
{
  std::vector<mars::common::Id> resIds;

  resIds.reserve(pIds.size());

  for (std::size_t i = 0; i < pIds.size(); ++i)
  {
    if(pIds[i].empty()) 
    resIds.push_back(mars::common::Id());
    else
    resIds.push_back(mars::common::Id(pIds[i]));
  }
  return resIds;
}