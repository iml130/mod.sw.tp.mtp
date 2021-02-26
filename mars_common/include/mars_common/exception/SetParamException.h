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

#ifndef MARS_COMMON_SETPARAMEXCEPTION_H
#define MARS_COMMON_SETPARAMEXCEPTION_H

#include <string>

namespace mars
{
namespace common
{
namespace exception
{
/**
 * @brief The SetParamException class
 */
class SetParamException : public std::exception
{
public:
  SetParamException(const std::string& msg) : msg(msg) {}

  ~SetParamException(void) {}

  virtual const char* what() const throw() { return msg.c_str(); }

private:
  std::string msg;

};
} // namespace exception
}  // namespace common
}  // namespace mars

#endif // MARS_COMMON_SETPARAMEXCEPTION_H
