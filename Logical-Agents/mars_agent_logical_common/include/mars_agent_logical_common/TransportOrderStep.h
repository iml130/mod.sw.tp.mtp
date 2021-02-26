#ifndef TRANSPORTORDERSTEP_H
#define TRANSPORTORDERSTEP_H

#include "mars_agent_logical_common/MoveOrder.h"
#include "mars_agent_logical_common/Order.h"
#include "mars_agent_logical_common/RobotAction.h"
#include <mars_agent_logical_msgs/TransportOrderStep.h>
#include <mars_common/Id.h>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
class TransportOrderStep : public mars::agent::logical::common::Order
{
public:
  TransportOrderStep(
      const mars_agent_logical_msgs::TransportOrderStep& transportOrderStepMsg);

  std::shared_ptr<mars::routing::common::topology::Entity>
  getDestination() const;

  const ros::Duration& getDestinationReservationDuration() const;

  const mars::agent::logical::common::MoveOrder& getMoveOrder() const;

  const mars::agent::logical::common::RobotAction& getRobotAction() const;

private:
  /**
   * @brief mMoveOrder Movement part of the transport order step
   */
  mars::agent::logical::common::MoveOrder mMoveOrder;

  mars::agent::logical::common::RobotAction mRobotAction;
};

} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // TRANSPORTORDERSTEP_H
