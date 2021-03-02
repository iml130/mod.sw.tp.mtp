//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//


#ifndef CONTAINERHANDLE_H
#define CONTAINERHANDLE_H

// std c++ includes
#include <map>
#include <memory>
#include <string>
#include <utility>

// own exception include
#include <mars_common/exception/SetParamException.h>

// ros msgs includes
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

// own includes
#include "ContainerVisualization.h"
#include <mars_common/Id.h>
#include <mars_topology_common/TopologyEntity.h>
#include <mars_topology_edge/MarsEdge.h>
#include <mars_topology_vertex/MarsVertex.h>
#include <mars_topology_vertex/MarsVertex.h>

namespace mars
{
namespace topology
{
namespace container
{
/**
 * This class represents an amount of entities (Vertex or/and Edge) of a topological graph. In this
 * case, an object of this class represents that groupe of entities saved as an unordered map.
 *
 */
class ContainerHandle
{
public:

  typedef std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::common::TopologyEntity>>::iterator iterator;
  typedef std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::common::TopologyEntity>>::const_iterator const_iterator;
  /**
   * @brief MarsContainer is initialized with empty mContainer
   */
  ContainerHandle();
  /**
   * @brief ~MarsContainer Deletes an object Edge object.
   */
  ~ContainerHandle();

  /**
   * @brief setId add an id as as a specifier of the Container.
   * @parm pId is a given id ,that should be given to identify  the Container.
   */
  void setId(const std::string& pId, const std::string& description);

  mars::common::Id getId(void) const;

  std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::common::TopologyEntity>>
  getContainer(void) const;
  /**
   * @brief addVertex add a vertex as an Entity to the Container.
   * @parm pMarsVertex is a given vertex ,that should be added to the Container.
   */
  void addVertex(std::shared_ptr<mars::topology::vertex::MarsVertex> pMarsVertex);
  /**
   * @brief addEdge add a edge as an Entity to the Container.
   * @parm pMarsEdge is a given edge ,that should be added to the Container.
   */
  void addEdge(std::shared_ptr<mars::topology::edge::MarsEdge> pMarsEdge);

  /**
   * @brief getEntity finds an entity , returns a pointer of
   * that element (when found)
   * @param as input the key(id in this case) should be given.
   */
  std::shared_ptr<mars::topology::common::TopologyEntity>
  getEntity(const mars::common::Id& pId) const;
  /**
   * @brief getVertex finds an entity of type vertex returns a pointer of
   * that element (when found)
   * @param as input the key(id in this case) should be given.
   */
  std::shared_ptr<mars::topology::vertex::MarsVertex> getVertex(const mars::common::Id& pId);
  /**
   * @brief getEdge finds an entity of type edge within the container and returns a pointer of
   * that element (when found)
   * @param as input the key(id in this case) should be given.
   */
  std::shared_ptr<mars::topology::edge::MarsEdge> getEdge(const mars::common::Id& pId);

  /**
   *@brief isVertex helps to indicate either the entity is a vertex or not
   *@param as input the key(id in this case) should be given.
   */
  bool isVertex(const mars::common::Id& pId);
  /**
   *@brief isEdge helps to indicate either the entity is a edge or not
   *@param as input the key(id in this case) should be given.
   */
  bool isEdge(const mars::common::Id& pId);

  /**
   * @brief draw helps drawing  footprints and directions of the entities in the container
   *
   */
  void draw(const ros::Publisher& markerPub, std::string frameId, const std::string& nodeNamespace);

  iterator begin();

  const_iterator begin() const;

  iterator end();
  
  const_iterator end() const;

private:
  mars::topology::container::ContainerVisualization mVisualization;

  /**
   * @brief mContainer the container that envelops the entities
   */
  std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::common::TopologyEntity>>
      mContainer;

  /**
   * @brief mId The Id of the container
   */
  mars::common::Id mId;
};
} // namespace container
} // namespace topology
} // namespace mars
#endif // MARS_TOPOLOGY_CONTAINER_CONTAINERHANDLE_H