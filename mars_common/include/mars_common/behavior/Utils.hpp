#ifndef UTILS_HPP
#define UTILS_HPP

// Behavior tree
#include <behaviortree_cpp_v3/behavior_tree.h>
// Logger
#include "mars_common/Logger.h"

namespace mars
{
namespace common
{
namespace behavior
{

template <typename T> bool setOutput(std::string parameterName, T& parameter)
{
  BT::Result setOutputResult;
  bool successful = true;

  setOutputResult = setOutput<T>(parameterName, parameter);

  if (!setOutputResult)
  {
    MARS_LOG_ERROR(setOutputResult.error());
    successful = false;
  }

  return successful;
}

} // namespace behavior
} // namespace common
} // namespace mars

#endif // UTILS_HPP
