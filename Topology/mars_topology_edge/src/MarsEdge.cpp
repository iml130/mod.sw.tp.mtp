#include "mars_topology_edge/MarsEdge.h"

mars::topology::edge::MarsEdge::MarsEdge(
    const std::string id, const std::string& description, int type, bool isLocked,
    float edgeCoordinateX, float edgeCoordinateY, const std::vector<float>& footprintX,
    const std::vector<float>& footprintY, const std::string& frameId, const std::string& originId,
    const std::string& originContainer, const std::string& targetId,
    const std::string& targetContainer, int pDirection, float edgeLength,
    const mars::topology::common::TopologyEntityRestrictions& pRestrictions)
    : mars::topology::common::TopologyEntity(id, description, type, isLocked, edgeCoordinateX,
                                             edgeCoordinateY, footprintX, footprintY, frameId,
                                             pRestrictions)
{
  this->mOriginId = mars::common::Id(originId, "");
  this->mTargetId = mars::common::Id(targetId, "");
  if (!originContainer.empty())
    this->mOriginContainer = mars::common::Id(originContainer, "");
  else
    this->mOriginContainer = mars::common::Id();
  if (!targetContainer.empty())
    this->mTargetContainer = mars::common::Id(targetContainer, "");
  else
    this->mTargetContainer = mars::common::Id();
  this->mLength = edgeLength;
  this->mDirection = mars::topology::common::DirectionType(pDirection);
}

const mars::common::Id& mars::topology::edge::MarsEdge::getOriginId(void) const
{
  return this->mOriginId;
}

const mars::common::Id& mars::topology::edge::MarsEdge::getTargetId(void) const
{
  return this->mTargetId;
}

const mars::common::Id& mars::topology::edge::MarsEdge::getOriginContainer(void) const
{
  return this->mOriginContainer;
}

const mars::common::Id& mars::topology::edge::MarsEdge::getTargetContainer(void) const
{
  return this->mTargetContainer;
}

float mars::topology::edge::MarsEdge::getLength(void) const { return this->mLength; }

mars::topology::common::DirectionType mars::topology::edge::MarsEdge::getDirection(void) const
{
  return this->mDirection;
}
