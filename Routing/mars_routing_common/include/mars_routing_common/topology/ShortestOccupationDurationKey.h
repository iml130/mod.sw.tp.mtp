#ifndef MARS_ROUTING_COMMON_TOPOLOGY_SHORTESTOCCUPATIONDURATIONHASHKEY_H
#define MARS_ROUTING_COMMON_TOPOLOGY_SHORTESTOCCUPATIONDURATIONHASHKEY_H

#include "mars_agent_physical_common/RobotAgentProperties.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
template <class Origin>
class ShortestOccupationDurationKey
{
public:
  ShortestOccupationDurationKey(const Origin& pOrigin, const mars::agent::physical::common::RobotAgentProperties& pRAP,
                                const double& pVelocity);

  ShortestOccupationDurationKey(
      const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey);

  ~ShortestOccupationDurationKey();

  ShortestOccupationDurationKey&
  operator=(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey) = delete;

  const Origin* getOrigin() const;
  const mars::agent::physical::common::RobotAgentProperties& getRobotAgentProperties() const;
  const double& getVelocity() const;

  bool operator==(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pOtherKey) const;

  struct Hash
  {
    std::size_t operator()(const mars::routing::common::topology::ShortestOccupationDurationKey<Origin>& pKey) const;
  };

private:
  const Origin* mOrigin;
  const mars::agent::physical::common::RobotAgentProperties mRAP;
  const double mVelocity;
};
}  // namespace topology
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_TOPOLOGY_SHORTESTOCCUPATIONDURATIONHASHKEY_H