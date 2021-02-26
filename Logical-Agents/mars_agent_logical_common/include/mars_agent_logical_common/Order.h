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

#ifndef MARS_AGENT_LOGICAL_COMMON_ORDER_H
#define MARS_AGENT_LOGICAL_COMMON_ORDER_H

#include "mars_common/Id.h"
#include "mars_common/Logger.h"
#include "mars_topology_common/TopologyEntity.h"

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
/**
 * @brief The order class serves as a superclass for any kind of orders.
 */
class Order
{
public:
  Order(const mars::common::Id& pOrderId);

  virtual ~Order();

  mars::common::Id getId(void) const;

  ros::Time getCreationTime(void) const;
  ros::Time getStartTime(void) const;
  ros::Time getCompletionTime(void) const;

  bool isCompleted(void) const;

protected:
  mars::common::Id mId;

  ros::Time mCreationTime;
  ros::Time mStartTime;
  ros::Time mCompletionTime;
};
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_COMMON_ORDER_H
