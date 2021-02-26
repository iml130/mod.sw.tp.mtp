/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : mars_agent_robot
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#include "mars_topology_common/TopologyEntityType.h"

#include <mars_topology_srvs/GetTypeResponse.h>

static const int TOPOLOGY_ENTITY_TYPE_UNKNOWN =
    mars_topology_msgs::TopologyEntityType::TOPOLOGY_ENTITY_TYPE_UNKNOWN;
static const int TOPOLOGY_ENTITY_TYPE_VERTEX_WAYPOINT =
    mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_WAYPOINT;
static const int TOPOLOGY_ENTITY_TYPE_VERTEX_PICKING_STATION =
    mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_PICKING_STATION;
static const int TOPOLOGY_ENTITY_TYPE_VERTEX_PARKING_LOT =
    mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_PARKING_LOT;
static const int TOPOLOGY_ENTITY_TYPE_EDGE =
    mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_EDGE;

static const std::string TOPOLOGY_ENTITY_TYPE_NAME_UNKNOWN = "UNKNOWN";
static const std::string TOPOLOGY_ENTITY_TYPE_NAME_VERTEX_WAYPOINT = "VERTEX_WAYPOINT";
static const std::string TOPOLOGY_ENTITY_TYPE_NAME_VERTEX_PICKING_STATION =
    "VERTEX_PICKING_STATION";
static const std::string TOPOLOGY_ENTITY_TYPE_NAME_VERTEX_PARKING_LOT = "VERTEX_PARKING_LOT";
static const std::string TOPOLOGY_ENTITY_TYPE_NAME_EDGE = "EDGE";

mars::topology::common::TopologyEntityType::TopologyEntityType(int pType) noexcept(false)
{
  this->setType(pType);
}

int mars::topology::common::TopologyEntityType::getType() const { return this->mType; }

void mars::topology::common::TopologyEntityType::setType(int pType) noexcept(false)
{
  switch (pType)
  {
  case TOPOLOGY_ENTITY_TYPE_VERTEX_WAYPOINT:
    this->mType = TOPOLOGY_ENTITY_TYPE_VERTEX_WAYPOINT;
    break;
  case TOPOLOGY_ENTITY_TYPE_VERTEX_PICKING_STATION:
    this->mType = TOPOLOGY_ENTITY_TYPE_VERTEX_PICKING_STATION;
    break;
  case TOPOLOGY_ENTITY_TYPE_VERTEX_PARKING_LOT:
    this->mType = TOPOLOGY_ENTITY_TYPE_VERTEX_PARKING_LOT;
    break;
  case TOPOLOGY_ENTITY_TYPE_EDGE:
    this->mType = TOPOLOGY_ENTITY_TYPE_EDGE;
    break;
  default:
    this->mType = TOPOLOGY_ENTITY_TYPE_UNKNOWN;

    throw mars::common::exception::SetParamException("Unknown topology entity type given: " +
                                                     std::to_string(pType));
    break;
  }
}

bool mars::topology::common::TopologyEntityType::operator==(TopologyEntityType otherType)
{
  return (this->mType == otherType.getType());
}

std::string std::to_string(const mars::topology::common::TopologyEntityType& pTopologyEntityType)
{
  std::string typeName;

  switch (pTopologyEntityType.getType())
  {
  case TOPOLOGY_ENTITY_TYPE_VERTEX_WAYPOINT:
    typeName = TOPOLOGY_ENTITY_TYPE_NAME_VERTEX_WAYPOINT;
    break;
  case TOPOLOGY_ENTITY_TYPE_VERTEX_PICKING_STATION:
    typeName = TOPOLOGY_ENTITY_TYPE_NAME_VERTEX_PICKING_STATION;
    break;
  case TOPOLOGY_ENTITY_TYPE_VERTEX_PARKING_LOT:
    typeName = TOPOLOGY_ENTITY_TYPE_NAME_VERTEX_PARKING_LOT;
    break;
  case TOPOLOGY_ENTITY_TYPE_EDGE:
    typeName = TOPOLOGY_ENTITY_TYPE_NAME_EDGE;
    break;
  default:
    typeName = TOPOLOGY_ENTITY_TYPE_NAME_UNKNOWN;
    break;
  }

  return typeName;
}
