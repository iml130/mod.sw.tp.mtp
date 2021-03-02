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

#ifndef MARS_ROUTING_PLUGIN_CARP_ROUTER_H
#define MARS_ROUTING_PLUGIN_CARP_ROUTER_H

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "mars_routing_core/Router.h"
#include "mars_routing_plugin_carp/PlanningRoute.h"
#include "mars_routing_srvs/GetRoute.h"

namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
class Router : public mars::routing::core::Router
{
public:
  Router();
  void initialize(ros::console::levels::Level pLogLevel);
  bool getRoute(mars_routing_srvs::GetRoute::Request& req, mars_routing_srvs::GetRoute::Response& res);

private:
  bool reserveRoute(mars::routing::plugin::carp::PlanningRoute& pRoute, const mars::common::Id& pAgentId) const;
};
}  // namespace carp
}  // namespace plugin
}  // namespace routing
}  // namespace mars

#endif  // MARS_ROUTING_PLUGIN_CARP_ROUTER_H
