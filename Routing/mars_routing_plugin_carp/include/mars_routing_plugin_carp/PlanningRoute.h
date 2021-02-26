#ifndef MARS_ROUTING_PLUGIN_CARP_PLANNINGROUTE_H
#define MARS_ROUTING_PLUGIN_CARP_PLANNINGROUTE_H

#include "mars_common/Id.h"

namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
/**
 * @class PlanningRoute
 */
class PlanningRoute
{
    public:
        virtual bool reserve(const mars::common::Id& pAgentId) = 0;
    protected:
    private:
};
}  // namespace carp
}  // namespace plugin
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_PLUGIN_CARP_PLANNINGROUTE_H