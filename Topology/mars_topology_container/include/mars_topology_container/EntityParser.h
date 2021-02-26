#ifndef ENTITYPARSER_H
#define ENTITYPARSER_H

#include "mars_topology_container/ContainerHandle.h"

// MARS includes
#include <mars_common/Id.h>
#include <mars_common/Logger.h>
#include <mars_common/exception/ReadParamException.h>

#include <mars_topology_edge/MarsEdge.h>
#include <mars_topology_vertex/MarsVertex.h>

#include <mars_topology_common/TopologyEntityRestrictions.h>

// ros includes
#include <ros/console.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h> // catkin component

namespace mars
{
namespace topology
{
namespace container
{
class EntityParser
{
public:
  // TODO change return types
  EntityParser();

  void parse(XmlRpc::XmlRpcValue& pEntities, mars::topology::container::ContainerHandle& pContainer);

private:
  /**
   * @brief getParam Reads the named paramter form the parameter server.
   * If the parameter could not be found a default value is assigned.
   */
  template <typename T>
  const T getParam(XmlRpc::XmlRpcValue& pEntity,
                   const std::string& pParamName) const noexcept(false);

  template <typename T>
  const std::vector<T> getParamVector(XmlRpc::XmlRpcValue& pEntity,
                                      const std::string& pParamName) const
      noexcept(false);

  template <typename T>
  const T getParam(XmlRpc::XmlRpcValue& pEntity, const std::string& pParamName,
                   const T& pDefaultValue) const;

  std::shared_ptr<mars::topology::vertex::MarsVertex> parseVertex(XmlRpc::XmlRpcValue& pVertex);

  std::shared_ptr<mars::topology::edge::MarsEdge> parseEdge(XmlRpc::XmlRpcValue& pEdge);
};
} // namespace container
} // namespace topology
} // namespace mars
#endif // ENTITYPARSER_H
