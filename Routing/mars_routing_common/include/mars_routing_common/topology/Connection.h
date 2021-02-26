#ifndef MARS_ROUTING_COMMON_TOPOLOGY_CONNECTION_H
#define MARS_ROUTING_COMMON_TOPOLOGY_CONNECTION_H

#include "mars_common/Id.h"
#include "mars_common/TimeInterval.h"

#include "mars_routing_common/topology/Vertex.h"
#include "mars_topology_msgs/Connection.h"

#include <eigen3/Eigen/Core>

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
class Connection
{
public:
  /**
   * @brief Direct constructor for a connection between two topology vertices
   * @param pOrigin Origin vertex of the connection
   * @param pDestination Destination vertex of the connection
   */
  Connection(mars::routing::common::topology::Vertex& pOrigin, mars::routing::common::topology::Vertex& pDestination);

  /**
   * @brief Constructor from Connection message
   * @param pMessage Ingoing connection message to construct from
   */
  Connection(mars_topology_msgs::Connection& pMessage);

  /**
   * @brief Primitive getter for mOrigin
   * @return The origin vertex of the connection
   */
  mars::routing::common::topology::Vertex& getOrigin();

  /**
   * @brief Primitive getter for mDestination
   * @return The destination vertex of the connection
   */
  mars::routing::common::topology::Vertex& getDestination();

private:
  mars::routing::common::topology::Vertex mOrigin;
  mars::routing::common::topology::Vertex mDestination;
};
}  // namespace topology
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_COMMON_TOPOLOGY_CONNECTION_H
