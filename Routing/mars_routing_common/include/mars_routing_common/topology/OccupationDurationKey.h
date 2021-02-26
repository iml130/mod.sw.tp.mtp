#ifndef MARS_ROUTING_COMMON_TOPOLOGY_OCCUPATIONDURATIONHASHKEY_H
#define MARS_ROUTING_COMMON_TOPOLOGY_OCCUPATIONDURATIONHASHKEY_H

#include "mars_agent_physical_common/RobotAgentProperties.h"

#include "mars_routing_common/topology/Entity.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
template <class Target>
class OccupationDurationKey
{
public:
  OccupationDurationKey(const Target& pOrigin, const Target& pTarget,
                        const mars::agent::physical::common::RobotAgentProperties& pRAP, const double& pVelocity);

  OccupationDurationKey(const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey);

  ~OccupationDurationKey();

  OccupationDurationKey&
  operator=(const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey) = delete;

  const Target* getOrigin() const;
  const Target* getTarget() const;
  const mars::agent::physical::common::RobotAgentProperties& getRobotAgentProperties() const;
  const double& getVelocity() const;

  bool operator==(const mars::routing::common::topology::OccupationDurationKey<Target>& pOtherKey) const;

  struct Hash
  {
    std::size_t operator()(const mars::routing::common::topology::OccupationDurationKey<Target>& pKey) const;
  };

private:
  const Target* mOrigin;
  const Target* mTarget;
  const mars::agent::physical::common::RobotAgentProperties mRAP;
  const double mVelocity;
};
}  // namespace topology
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_TOPOLOGY_OCCUPATIONDURATIONHASHKEY_H