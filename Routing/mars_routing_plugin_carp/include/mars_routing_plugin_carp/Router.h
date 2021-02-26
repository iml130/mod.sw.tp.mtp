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
