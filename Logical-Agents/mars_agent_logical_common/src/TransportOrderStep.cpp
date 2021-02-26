#include "mars_agent_logical_common/TransportOrderStep.h"

mars::agent::logical::common::TransportOrderStep::TransportOrderStep(
    const mars_agent_logical_msgs::TransportOrderStep& transportOrderStepMsg)
    : mars::agent::logical::common::Order(
          transportOrderStepMsg.transport_order_step_id), mMoveOrder(transportOrderStepMsg.move_order), mRobotAction(transportOrderStepMsg.robot_action)
{
}

std::shared_ptr<mars::routing::common::topology::Entity>
mars::agent::logical::common::TransportOrderStep::getDestination() const
{
  return this->mMoveOrder.getDestination();
}

const ros::Duration& mars::agent::logical::common::TransportOrderStep::
    getDestinationReservationDuration() const
{
  return this->mMoveOrder.getDestinationReservationDuration();
}

const mars::agent::logical::common::RobotAction& mars::agent::logical::common::TransportOrderStep::getRobotAction() const
{
  return mRobotAction;
}

const mars::agent::logical::common::MoveOrder& mars::agent::logical::common::TransportOrderStep::getMoveOrder() const
{
  return mMoveOrder;
}
