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

#ifndef MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYTYPE_H
#define MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYTYPE_H

#include <mars_common/exception/SetParamException.h>

#include <string>

namespace mars
{
namespace topology
{
namespace common
{

enum class DirectionType : int
{
  unidirectional = 0,
  unidirectional_reversed = 1,
  bidirectional = 2
};

class TopologyEntityType
{
public:

  /**
   * @brief Construct a new Topology Entity Type object
   * 
   * @param pType 
   * @throw mars::common::exception::SetParamException
   */
  TopologyEntityType(int pType) noexcept(false);

  int getType(void) const;

  bool operator==(TopologyEntityType otherType);

private:
  int mType;

  /**
   * @brief Set the Type object
   * 
   * @param pType 
   * @throw mars::common::exception::SetParamException
   */
  void setType(int pType) noexcept(false);
};
}
}
}

namespace std
{
string to_string(const mars::topology::common::TopologyEntityType& pTopologyEntityType);
}

#endif // MARS_TOPOLOGY_COMMON_TOPOLOGYENTITYTYPE_H
