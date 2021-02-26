#include "mars_routing_common/topology/Entity.h"

mars::routing::common::topology::Entity::Entity(
    const mars::common::Id& pId)
    : mId(pId)
{
}

mars::routing::common::topology::Entity::~Entity() {}

const mars::common::Id& mars::routing::common::topology::Entity::getId() const
{
  return this->mId;
}

bool mars::routing::common::topology::Entity::
operator==(const Entity& pComparedEntity) const
{
  return (this->getId() == pComparedEntity.getId());
};

bool mars::routing::common::topology::Entity::
operator!=(const Entity& pComparedEntity) const
{
  return !(*this == pComparedEntity);
};
