/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : TODO: INSERT PROJECT NAME
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Marc Down, Max Mustermann
 * Contact(s)       : marc.down@iml.fraunhofer.de
 *                    max.mustermann@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#ifndef MARS_TOPOLOGY_VERTEX_MARSVERTEX_H
#define MARS_TOPOLOGY_VERTEX_MARSVERTEX_H

#include "mars_topology_common/TopologyEntityLock.h"
#include "mars_topology_common/TopologyEntityReservation.h"

#include <mars_common/Id.h>
#include <mars_common/exception/SetParamException.h>
#include <mars_topology_common/TopologyEntity.h>
#include <mars_topology_common/TopologyEntityRestrictions.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <vector>

namespace mars
{
namespace topology
{
namespace vertex
{
class MarsVertex : public mars::topology::common::TopologyEntity
{
public:
  MarsVertex(const std::string id, const std::string& description, int type, bool isLocked,
             float vertexCoordinateX, float vertexCoordinateY, const std::vector<float>& footprintX,
             const std::vector<float>& footprintY, const std::string& frameId,
             const std::vector<mars::common::Id>& outgoingEdgeIds,
             const std::vector<std::string>& outgoingContainerIds,
             const std::vector<mars::common::Id>& ingoingEdgeIds,
             const std::vector<std::string>& ingoingContainerIds,
             const mars::topology::common::TopologyEntityRestrictions& pRestrictions);

  ~MarsVertex();

  std::vector<mars::common::Id> getIngoingEdgeIds(void) const;

  std::vector<mars::common::Id> getOutgoingEdgeIds(void) const;

  std::vector<mars::common::Id> getIngoingContainerIds(void) const;

  std::vector<mars::common::Id> getOutgoingContainerIds(void) const;

private:
// this differs from the mars::common::Id::setIds. This last one cannot initialize empty ids.  
  std::vector<mars::common::Id> setIds(std::vector<std::string> pIds) const;

  std::vector<mars::common::Id> mOutgoingEdgeIds;
  std::vector<mars::common::Id> mIngoingEdgeIds;
  std::vector<mars::common::Id> mOutgoingContainerIds;
  std::vector<mars::common::Id> mIngoingContainerIds;
};
} // namespace vertex
} // namespace topology
} // namespace mars
#endif // MARS_TOPOLOGY_VERTEX_MARSVERTEX_H
