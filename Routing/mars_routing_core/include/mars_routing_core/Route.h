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

#ifndef MARS_ROUTING_CORE_ROUTE_H
#define MARS_ROUTING_CORE_ROUTE_H

#include "mars_routing_common/topology/Entity.h"

#include "mars_routing_core/IterationRoute.h"
#include "mars_routing_core/Step.h"
#include "mars_routing_core/exception/InvalidRouteException.hpp"

#include "mars_routing_srvs/GetRoute.h"

#include <vector>

namespace mars
{
namespace routing
{
namespace core
{

/**
 * @class Route
 */
template <class Origin, class Target, class Destination> class Route : public IterationRoute
{
public:
  typedef RouteIterator iterator;
  typedef const RouteIterator const_iterator;

  /**
   * @brief Constructs a Route from a GetRoute service request and response.
   * @param pServiceResponse GetRoute service with a filled out valid route.
   */
  Route(const mars_routing_srvs::GetRoute& pService);

  /**
   * @brief Destructs the Route by freeing steps in doubly linked list
   */
  virtual ~Route();

  /**
   * @brief Primitive getter for mId.
   * @return Unique Identifier of the Route.
   */
  const mars::common::Id& getId() const;

  /**
   * @brief Merges this route into a GetRoute service response to send.
   * @param pServiceResponse Service object to merge this route into.
   */
  void mergeIntoServiceResponse(mars_routing_srvs::GetRouteResponse& pServiceResponse) const;

  /**
   * @brief getTravelInterval
   * @return The calculated time interval it will take for a robot to travel the entire route.
   * @throw mars::routing::core::exception::InvalidRouteException
   */
  const mars::common::TimeInterval& getTravelInterval() const noexcept(false);

  /**
   * @brief getTravelDistance
   * @return The total travel distance of the route.
   * @throw mars::routing::core::exception::InvalidRouteException
   */
  double getTravelDistance() const noexcept(false);

  /**
   * @brief getOrigin
   * @return The origin of the route.
   */
  Origin& getOrigin();

  /**
   * @brief getDestination
   * @return The destination of the route.
   */
  Destination& getDestination();

  unsigned int getStepCount() const;

  /**
   * @brief isValid
   * @return if the route is valid.
   */
  bool isValid() const;

  /**
   * @brief begin
   * @return Pointer to first step iterator of the Route.
   * @throw mars::routing::core::exception::InvalidRouteException
   */
  iterator begin() const noexcept(false);

  /**
   * @brief end
   * @return Pointer to past last step of the Route.
   */
  iterator end() const;

  /**
   * @brief Create visualization marker for this
   * 
   * @param pStartTime The start time, relative to which this is visualized. Defines the time that corresponds to a route at ground level.
   * @return visualization_msgs::Marker visualization message representation of this
   */
 visualization_msgs::Marker visualize(const ros::Time& pStartTime);

protected:
  Route(Origin& pOrigin, Destination& pDestination);

  mars::common::Id mId;

  Origin mOrigin;
  Destination mDestination;

  mars::common::TimeInterval mTravelInterval;
  double mTravelDistance;

  bool mValid;
  unsigned int mStepCount;

  /**
   * @brief Flag indicating wheter visualization has been initialized 
   */
  bool mVisualizationInitialized;

  /**
   * @brief Incremented ID needed to visually differentiate between routes (e.g. determines color) 
   */
  int mVisualizationID;

  /**
   * @brief Hue is the angular component in the conical HSV color space with values in [0, 360) degree.
   */
  double mHueDegree;

  /**
   * @brief Pointer to first step in the route 
   */
  mars::routing::core::Step<Origin, Target>* mFirstStep;

  /**
   * @brief Pointer to last step in the route
   */
  mars::routing::core::Step<Origin, Target>* mLastStep;

  /**
   * @brief Static variables to hold information about the point of time when the (globally, across all route objects) last step is planned
   */
  static ros::Time sLatestTime;

  /**
   * @brief Global route visualization ID 
   */
  static int sVisualizationID;
};
} // namespace core
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_CORE_ROUTE_H
