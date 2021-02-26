#ifndef MARS_ROUTING_PLUGIN_CARP_UTILITIES_H
#define MARS_ROUTING_PLUGIN_CARP_UTILITIES_H

#include <boost/optional.hpp>

#include "mars_common/TimeInterval.h"

namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
class utilities // static
{
public:
  static boost::optional<ros::Time>
  getReservationStartTime(const mars::common::TimeInterval& pTargetFreeTimeInterval,
                          const ros::Time& lTargetEntryTime,
                          const ros::Duration& pLeastTargetOccupationDuration);

private:
  utilities();

  static ros::Duration
  calcLeastReservationDuration(const mars::common::TimeInterval& pTargetFreeTimeInterval,
                               const ros::Time& pTargetEntryTime,
                               const ros::Duration& pLeastTargetOccupationDuration);
};
} // namespace carp
} // namespace plugin
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_PLUGIN_CARP_UTILITIES_H
