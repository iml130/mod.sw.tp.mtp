/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : MARS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * --------------------------------------------------------------------------------------------- */

#ifndef MARS_COMMON_BEHAVIOR_COUTDURATIONLOGGER_H
#define MARS_COMMON_BEHAVIOR_COUTDURATIONLOGGER_H

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <cstring>
#include <map>
#include <chrono>

namespace BT
{

enum ActionTransition
{
  UNKNOWN,
};

class CoutDurationLogger : public StatusChangeLogger
{

  static std::atomic<bool> gRefCount;

public:
  CoutDurationLogger(const Tree& pTree);
  ~CoutDurationLogger() override;

  void callback(Duration pTimestamp, const TreeNode& pNode, NodeStatus pPrevStatus,
                NodeStatus pStatus) override;

  void flush() override{};

private:
  std::map<std::string, std::chrono::high_resolution_clock::time_point> mStartTimes;

  std::chrono::duration<double> handleTransition(const TreeNode& pNode);
};

} // namespace BT

#endif // MARS_COMMON_BEHAVIOR_COUTDURATIONLOGGER_H
