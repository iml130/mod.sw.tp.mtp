#include "mars_agent_logical_common/TransportOrder.h"

mars::agent::logical::common::TransportOrder::TransportOrder(
    const mars_agent_logical_msgs::TransportOrder& transportOrderMsg)
    : mars::agent::logical::common::Order(transportOrderMsg.transport_order_id), mStartStep(transportOrderMsg.start_step), mDestinationStep(transportOrderMsg.destination_step)
{
}

const mars::agent::logical::common::TransportOrderStep&
mars::agent::logical::common::TransportOrder::getDestinationStep() const
{
  return mDestinationStep;
}

const mars::agent::logical::common::TransportOrderStep&
mars::agent::logical::common::TransportOrder::getStartStep() const
{
  return mStartStep;
}
