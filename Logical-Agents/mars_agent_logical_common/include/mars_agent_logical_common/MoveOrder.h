/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : MARS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#ifndef MARS_AGENT_LOGICAL_COMMON_MOVEORDER_H
#define MARS_AGENT_LOGICAL_COMMON_MOVEORDER_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "mars_agent_logical_common/Order.h"
#include "mars_agent_logical_msgs/MoveOrder.h"
#include "mars_common/Logger.h"
#include "mars_common/Id.h"
#include "mars_routing_common/topology/Entity.h"

#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
/**
 * @brief The MoveOrder class
 */
class MoveOrder : public mars::agent::logical::common::Order
{
public:
  MoveOrder(const mars_agent_logical_msgs::MoveOrder& pMoveOrderMsg);

  ~MoveOrder();

  std::shared_ptr<mars::routing::common::topology::Entity> getDestination() const;
  const ros::Duration& getDestinationReservationDuration() const;

protected:
private:
  std::shared_ptr<mars::routing::common::topology::Entity> mDestination;
  ros::Duration mDestinationReservationDuration;
};
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_MOVEORDER_H
