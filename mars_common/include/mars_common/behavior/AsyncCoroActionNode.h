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
