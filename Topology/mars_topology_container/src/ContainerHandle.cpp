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

#include "mars_topology_container/ContainerHandle.h"

static const std::string EXEPTION_MSG_INVALID_GIVEN_ID = "the given Id is invalid od empty";
static const std::string EXCEPTION_MSG_INVALID_VERTEX = "the given parameter is not a vertex type";
static const std::string EXCEPTION_MSG_INVALID_EDGE = "the given parameter is not an edge type";
static const std::string EXEPTION_MSG_UNFOUND_ENTITY =
    "the given Id don't match with any of the registered Entity";
static const std::string Vertex_Entity = "vetrex_";
static const std::string Edge_Entity = "edge_";

mars::topology::container::ContainerHandle::ContainerHandle() { mContainer.clear(); }

mars::topology::container::ContainerHandle::~ContainerHandle() {}

void mars::topology::container::ContainerHandle::setId(const std::string& pId,
                                                       const std::string& description)
{
  this->mId.initialize(pId, description);
  this->mVisualization.setContainerId(this->mId);
}
mars::common::Id mars::topology::container::ContainerHandle::getId(void) const { return this->mId; }
std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::common::TopologyEntity>>
mars::topology::container::ContainerHandle::getContainer(void) const
{
  return this->mContainer;
}

void mars::topology::container::ContainerHandle::addVertex(
    std::shared_ptr<mars::topology::vertex::MarsVertex> pMarsVertex)
{
  this->mContainer.insert(std::make_pair(pMarsVertex->getId(), pMarsVertex));
}
void mars::topology::container::ContainerHandle::addEdge(
    std::shared_ptr<mars::topology::edge::MarsEdge> pMarsEdge)
{
  this->mContainer.insert(std::make_pair(pMarsEdge->getId(), pMarsEdge));
}

std::shared_ptr<mars::topology::common::TopologyEntity>
mars::topology::container::ContainerHandle::getEntity(const mars::common::Id& pId) const
{
  if (pId.isValid())
  {
    const auto& lEntity = this->mContainer.find(pId);
    if (lEntity == this->mContainer.end())
    {
      throw mars::common::exception::SetParamException(EXEPTION_MSG_UNFOUND_ENTITY);
    }
    return lEntity->second;
  }
  else
  {
    throw mars::common::exception::SetParamException(EXEPTION_MSG_INVALID_GIVEN_ID);
  }
}

std::shared_ptr<mars::topology::vertex::MarsVertex>
mars::topology::container::ContainerHandle::getVertex(const mars::common::Id& pId)
{
  if (pId.isValid())
  {
    const auto& lEntity = this->mContainer.find(pId);
    // at first the element has to be found.Then check if it is a vertex type
    if (lEntity != this->mContainer.end())
    {
      std::shared_ptr<mars::topology::vertex::MarsVertex> lVertex =
          std::dynamic_pointer_cast<mars::topology::vertex::MarsVertex>(lEntity->second);
      if (!lVertex)
      {
        // its not a vertex element
        throw mars::common::exception::SetParamException(EXCEPTION_MSG_INVALID_VERTEX);
      }
      return lVertex;
    }
  }
  else
  {
    throw mars::common::exception::SetParamException(EXEPTION_MSG_INVALID_GIVEN_ID);
  }
}

std::shared_ptr<mars::topology::edge::MarsEdge>
mars::topology::container::ContainerHandle::getEdge(const mars::common::Id& pId)
{
  if (pId.isValid())
  {
    const auto& lEntity = this->mContainer.find(pId);
    // at first the element has to be found.Then check if it is a edge type
    if (lEntity != this->mContainer.end())
    {
      std::shared_ptr<mars::topology::edge::MarsEdge> lEdge =
          std::dynamic_pointer_cast<mars::topology::edge::MarsEdge>(lEntity->second);
      if (!lEdge)
      {
        // its not an edge element
        throw mars::common::exception::SetParamException(EXCEPTION_MSG_INVALID_EDGE);
      }
      return lEdge;
    }
  }
  else
  {
    throw mars::common::exception::SetParamException(EXEPTION_MSG_INVALID_GIVEN_ID);
  }
}

bool mars::topology::container::ContainerHandle::isVertex(const mars::common::Id& pId)
{
  const auto& lEntity = this->mContainer.find(pId);
  // at first the element has to be found.Then check if it is a edge type
  if (lEntity != this->mContainer.end())
  {
    std::shared_ptr<mars::topology::vertex::MarsVertex> lVertex =
        std::dynamic_pointer_cast<mars::topology::vertex::MarsVertex>(lEntity->second);
    if (!lVertex)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

bool mars::topology::container::ContainerHandle::isEdge(const mars::common::Id& pId)
{
  const auto& lEntity = this->mContainer.find(pId);
  // at first the element has to be found.Then check if it is a edge type
  if (lEntity != this->mContainer.end())
  {
    std::shared_ptr<mars::topology::edge::MarsEdge> lEdge =
        std::dynamic_pointer_cast<mars::topology::edge::MarsEdge>(lEntity->second);
    if (!lEdge)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

void mars::topology::container::ContainerHandle::draw(const ros::Publisher& markerPub,
                                                      std::string frameId,
                                                      const std::string& nodeNamespace)
{
  std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::edge::MarsEdge>>
      lEdgesContainer;
  std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::vertex::MarsVertex>>
      lVerticesContainer;
  for (auto& it : this->mContainer)
  {
    if (this->isEdge(it.first))
    {
      lEdgesContainer.insert(std::make_pair(it.first, this->getEdge(it.first)));
    }
    else
    {
      lVerticesContainer.insert(std::make_pair(it.first, this->getVertex(it.first)));
    }
  }

  this->mVisualization.drawDirection(this->getId(), lEdgesContainer, this->getContainer(), frameId, nodeNamespace);
  this->mVisualization.drawVertex(lVerticesContainer, frameId);

  this->mVisualization.drawFootprint(this->getContainer(), markerPub, frameId);
}

mars::topology::container::ContainerHandle::iterator
mars::topology::container::ContainerHandle::begin()
{
  return this->mContainer.begin();
}

mars::topology::container::ContainerHandle::const_iterator
mars::topology::container::ContainerHandle::begin() const
{
  return this->mContainer.begin();
}

mars::topology::container::ContainerHandle::iterator
mars::topology::container::ContainerHandle::end()
{
  return this->mContainer.end();
}

mars::topology::container::ContainerHandle::const_iterator
mars::topology::container::ContainerHandle::end() const
{
  return this->mContainer.end();
}
