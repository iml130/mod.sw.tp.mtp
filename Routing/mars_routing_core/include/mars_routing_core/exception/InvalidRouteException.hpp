#ifndef MARS_ROUTING_BASE_EXCEPTION_INVALIDROUTEEXCEPTION_H
#define MARS_ROUTING_BASE_EXCEPTION_INVALIDROUTEEXCEPTION_H

#include <string>

namespace mars
{
namespace routing
{
namespace core
{
namespace exception
{
/**
 * @brief The InvalidRouteException class
 */
class InvalidRouteException : public std::exception
{
public:
  InvalidRouteException(const std::string& msg) : msg(msg) {}

  ~InvalidRouteException(void) {}

  virtual const char* what() const throw() { return msg.c_str(); }

private:
  std::string msg;
};
} // namespace exception
} // namespace base
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_BASE_EXCEPTION_INVALIDROUTEEXCEPTION_H
