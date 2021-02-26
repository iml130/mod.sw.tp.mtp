#include "mars_routing_common/topology/OccupationDurationKey.h"

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

template <class Target>
mars::routing::common::topology::OccupationDurationKey<Target>::OccupationDurationKey(
    const Target& pOrigin, const Target& pTarget, const mars::agent::physical::common::RobotAgentProperties& pRAP,
    const double& pVelocity)
  : mOrigin(new Target(pOrigin)), mTarget(new Target(pTarget)), mRAP(pRAP), mVelocity(pVelocity)
{
}

template <class Target>
mars::routing::common::topology::OccupationDurationKey<Target>::OccupationDurationKey(
    const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey)
  : mOrigin(new Target(*pOtherKey.getOrigin()))
  , mTarget(new Target(*pOtherKey.getTarget()))
  , mRAP(pOtherKey.getRobotAgentProperties())
  , mVelocity(pOtherKey.getVelocity())
{
}

template <class Target>
mars::routing::common::topology::OccupationDurationKey<Target>::~OccupationDurationKey()
{
  delete mOrigin;
  delete mTarget;
}

template <class Target>
const Target* mars::routing::common::topology::OccupationDurationKey<Target>::getOrigin() const
{
  return this->mOrigin;
}

template <class Target>
const Target* mars::routing::common::topology::OccupationDurationKey<Target>::getTarget() const
{
  return this->mTarget;
}

template <class Target>
const mars::agent::physical::common::RobotAgentProperties&
mars::routing::common::topology::OccupationDurationKey<Target>::getRobotAgentProperties() const
{
  return this->mRAP;
}

template <class Target>
const double& mars::routing::common::topology::OccupationDurationKey<Target>::getVelocity() const
{
  return this->mVelocity;
}

template <class Target>
bool mars::routing::common::topology::OccupationDurationKey<Target>::
operator==(const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey) const
{
  return this->getOrigin()->getId() == pOtherKey.getOrigin()->getId() &&
         this->getTarget()->getId() == pOtherKey.getTarget()->getId() &&
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

template <class Target>
std::size_t mars::routing::common::topology::OccupationDurationKey<Target>::Hash::
operator()(const mars::routing::common::topology::OccupationDurationKey<Target>& pKey) const
{
  std::size_t seed = 17;

  seed = seed * 31 + std::hash<mars::common::Id>()(pKey.getOrigin()->getId());
  seed = seed * 31 + std::hash<mars::common::Id>()(pKey.getTarget()->getId());
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

template class mars::routing::common::topology::OccupationDurationKey<mars::routing::common::topology::Edge>;
template class mars::routing::common::topology::OccupationDurationKey<mars::routing::common::topology::Vertex>;