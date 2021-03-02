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

#ifndef MARS_ROUTING_CORE_ITERATION_ROUTE_H
#define MARS_ROUTING_CORE_ITERATION_ROUTE_H

#include "mars_routing_common/topology/Entity.h"

#include "mars_routing_core/Step.h"

#include "mars_routing_srvs/GetRoute.h"

#include <vector>

#include <visualization_msgs/Marker.h>

namespace mars
{
namespace routing
{
namespace core
{
/**
 * @class RouteIterator
 */
class RouteIterator
{
  using iterator_category = std::forward_iterator_tag;
  using value_type = mars::routing::core::IterationStep;
  using difference_type = ptrdiff_t;
  using pointer = mars::routing::core::IterationStep*;
  using reference = mars::routing::core::IterationStep&;

public:
  RouteIterator(mars::routing::core::IterationStep* pStep = nullptr) { mStep = pStep; }

  RouteIterator(const RouteIterator& rawIterator) = default;

  ~RouteIterator() {}

  operator bool() const
  {
    if (mStep)
      return true;
    else
      return false;
  }

  RouteIterator& operator=(const RouteIterator& rawIterator) = default;
  RouteIterator& operator=(mars::routing::core::IterationStep* pStep)
  {
    mStep = pStep;
    return (*this);
  }

  bool operator==(const RouteIterator& pIterator) const
  {
    return (mStep == pIterator.getConstStep());
  }

  bool operator!=(const RouteIterator& pIterator) const
  {
    return (mStep != pIterator.getConstStep());
  }

  RouteIterator& operator++()
  {
    mStep = mStep->getNext();
    return (*this);
  }

  RouteIterator& operator--()
  {
    mStep = mStep->getPrevious();
    return (*this);
  }

  mars::routing::core::IterationStep& operator*() { return *mStep; }

  const mars::routing::core::IterationStep& operator*() const { return *mStep; }

  mars::routing::core::IterationStep* operator->() { return mStep; }

  mars::routing::core::IterationStep* getStep() const { return mStep; }

  const mars::routing::core::IterationStep* getConstStep() const { return mStep; }

protected:
  mars::routing::core::IterationStep* mStep;
};

class IterationRoute
{
public:
  typedef RouteIterator iterator;
  typedef const RouteIterator const_iterator;

  virtual ~IterationRoute(){};

  virtual const mars::common::Id& getId() const = 0;

  virtual void
  mergeIntoServiceResponse(mars_routing_srvs::GetRouteResponse& pServiceResponse) const = 0;

  virtual const mars::common::TimeInterval& getTravelInterval() const = 0;

  virtual double getTravelDistance() const = 0;

  virtual bool isValid() const = 0;

  virtual unsigned int getStepCount() const = 0;

  virtual iterator begin() const = 0;

  virtual iterator end() const = 0;

  virtual visualization_msgs::Marker visualize(const ros::Time& pStartTime) = 0;

};
} // namespace core
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_CORE_ITERATION_ROUTE_H