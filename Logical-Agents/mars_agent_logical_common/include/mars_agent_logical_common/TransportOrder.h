#ifndef TRANSPORTORDER_H
#define TRANSPORTORDER_H

#include "mars_agent_logical_common/Order.h"
#include "mars_agent_logical_common/TransportOrderStep.h"

#include <mars_agent_logical_msgs/TransportOrder.h>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
class TransportOrder : public mars::agent::logical::common::Order
{
public:
  TransportOrder(
      const mars_agent_logical_msgs::TransportOrder& transportOrderMsg);

  const mars::agent::logical::common::TransportOrderStep& getStartStep() const;

  const mars::agent::logical::common::TransportOrderStep&
  getDestinationStep() const;

private:
  mars::agent::logical::common::TransportOrderStep mStartStep;

  mars::agent::logical::common::TransportOrderStep mDestinationStep;
};
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // TRANSPORTORDER_H
