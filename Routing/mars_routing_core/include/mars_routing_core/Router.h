#ifndef MARS_ROUTING_CORE_ROUTER_H
#define MARS_ROUTING_CORE_ROUTER_H

#include "mars_common/Id.h"
#include "mars_routing_srvs/GetRoute.h"

#include <ros/ros.h>
#include <cstddef>
#include <vector>

namespace mars
{
namespace routing
{
namespace core
{
class Router
{
public:
  virtual void initialize(ros::console::levels::Level pLogLevel) = 0;
  virtual bool getRoute(mars_routing_srvs::GetRoute::Request &req, mars_routing_srvs::GetRoute::Response &res) = 0;
  virtual ~Router() {};

protected:
  Router() {};
};
} // namespace core
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_CORE_ROUTER_H
