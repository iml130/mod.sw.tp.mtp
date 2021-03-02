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
