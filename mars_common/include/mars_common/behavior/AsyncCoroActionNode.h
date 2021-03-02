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


#ifndef MARS_COMMON_BEHAVIOR_ASYNCCOROACTIONNODE_H
#define MARS_COMMON_BEHAVIOR_ASYNCCOROACTIONNODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

namespace BT
{
class AsyncCoroActionNode : public BT::ActionNodeBase
{
public:
  AsyncCoroActionNode(const std::string& pName, const BT::NodeConfiguration& pConfig);
  virtual ~AsyncCoroActionNode() override;

  virtual BT::NodeStatus executeTick() override final;

  void stopAndJoinThread();

private:
  void asyncThreadLoop();

  void waitStart();

  void notifyStart();

  std::atomic<bool> keep_thread_alive_;
  bool start_action_;
  std::mutex start_mutex_;
  std::condition_variable start_signal_;
  std::exception_ptr exptr_;
  std::thread thread_;

  /* CUSTOM ADJUSTMENTS */
protected:
  void waitForTick();

private:
  void notifyTick();

  mutable std::mutex mTickMutex;
  std::condition_variable mTickSignal;
};
} // namespace BT

#endif // MARS_COMMON_BEHAVIOR_ASYNCCOROACTIONNODE_H
