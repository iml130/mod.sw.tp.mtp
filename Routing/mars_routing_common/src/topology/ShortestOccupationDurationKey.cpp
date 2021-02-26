#include "mars_routing_common/topology/ShortestOccupationDurationKey.h"

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

template <class Origin>
mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::ShortestOccupationDurationKey(
    const Origin& pOrigin, const mars::agent::physical::common::RobotAgentProperties& pRAP, const double& pVelocity)
  : mOrigin(new Origin(pOrigin)), mRAP(pRAP), mVelocity(pVelocity){};

template <class Origin>
mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::ShortestOccupationDurationKey(
    const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey)
  : mOrigin(new Origin(*pOtherKey.getOrigin()))
  , mRAP(pOtherKey.getRobotAgentProperties())
  , mVelocity(pOtherKey.getVelocity())
{
}

template <class Origin>
mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::~ShortestOccupationDurationKey()
{
  delete mOrigin;
}

template <class Origin>
const Origin* mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::getOrigin() const
{
  return this->mOrigin;
}

template <class Origin>
const mars::agent::physical::common::RobotAgentProperties&
mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::getRobotAgentProperties() const
{
  return this->mRAP;
}

template <class Origin>
const double& mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::getVelocity() const
{
  return this->mVelocity;
}

template <class Origin>
bool mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::
operator==(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey) const
{
  return this->getOrigin()->getId() == pOtherKey.getOrigin()->getId() &&
         this->getRobotAgentProperties().getForwardVelocity() ==
             pOtherKey.getRobotAgentProperties().getForwardVelocity() &&
         this->getRobotAgentProperties().getLinearAcceleration() ==
             pOtherKey.getRobotAgentProperties().getLinearAcceleration() &&
         this->getRobotAgentProperties().getLinearDeceleration() ==
             pOtherKey.getRobotAgentProperties().getLinearDeceleration() &&
         this->getRobotAgentProperties().getAngularVelocity() ==
             pOtherKey.getRobotAgentProperties().getAngularVelocity() &&
         this->getRobotAgentProperties().getAngularAcceleration() ==
             pOtherKey.getRobotAgentProperties().getAngularAcceleration() &&
         this->getRobotAgentProperties().getAngularDeceleration() ==
             pOtherKey.getRobotAgentProperties().getAngularDeceleration() &&
         this->getRobotAgentProperties().getTurningRadius() == pOtherKey.getRobotAgentProperties().getTurningRadius() &&
         this->getVelocity() == pOtherKey.getVelocity();
};

template <class Origin>
std::size_t mars::routing::common::topology::ShortestOccupationDurationKey<Origin>::Hash::
operator()(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pKey) const
{
  std::size_t seed = 17;

  seed = seed * 31 + std::hash<mars::common::Id>()(pKey.getOrigin()->getId());
  seed = seed * 31 + std::hash<double>()(pKey.getRobotAgentProperties().getForwardVelocity());
  seed = seed * 31 + std::hash<double>()(pKey.getRobotAgentProperties().getLinearAcceleration());
  seed = seed * 31 + std::hash<double>()(pKey.getRobotAgentProperties().getLinearDeceleration());
  seed = seed * 31 + std::hash<double>()(pKey.getRobotAgentProperties().getAngularVelocity());
  seed = seed * 31 + std::hash<double>()(pKey.getRobotAgentProperties().getAngularAcceleration());
  seed = seed * 31 + std::hash<double>()(pKey.getRobotAgentProperties().getAngularDeceleration());
  seed = seed * 31 + std::hash<double>()(pKey.getRobotAgentProperties().getTurningRadius());
  seed = seed * 31 + std::hash<double>()(pKey.getVelocity());

  return seed;
};

template class mars::routing::common::topology::ShortestOccupationDurationKey<mars::routing::common::topology::Edge>;
template class mars::routing::common::topology::ShortestOccupationDurationKey<mars::routing::common::topology::Vertex>;