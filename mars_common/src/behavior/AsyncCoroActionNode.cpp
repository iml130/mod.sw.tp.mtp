#include "mars_common/behavior/AsyncCoroActionNode.h"

BT::AsyncCoroActionNode::AsyncCoroActionNode(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::ActionNodeBase(pName, pConfig), keep_thread_alive_(true), start_action_(false)
{
  thread_ = std::thread(&BT::AsyncCoroActionNode::asyncThreadLoop, this);
}

BT::AsyncCoroActionNode::~AsyncCoroActionNode()
{
  if (thread_.joinable())
  {
    stopAndJoinThread();
  }
}

void BT::AsyncCoroActionNode::waitStart()
{
  std::unique_lock<std::mutex> lock(start_mutex_);
  while (!start_action_)
  {
    start_signal_.wait(lock);
  }
  start_action_ = false;
}

void BT::AsyncCoroActionNode::notifyStart()
{
  std::unique_lock<std::mutex> lock(start_mutex_);
  start_action_ = true;
  start_signal_.notify_all();
}

void BT::AsyncCoroActionNode::asyncThreadLoop()
{
  while (keep_thread_alive_.load())
  {
    waitStart();

    // check keep_thread_alive_ again because the tick_engine_ could be
    // notified from the method stopAndJoinThread
    if (keep_thread_alive_)
    {
      // this will execute the blocking code.
      try
      {
        setStatus(tick());
      }
      catch (std::exception&)
      {
        std::cerr << "\nUncaught exception from the method tick() of an AsyncActionNode: ["
                  << registrationName() << "/" << name() << "]\n"
                  << std::endl;
        exptr_ = std::current_exception();
        keep_thread_alive_ = false;
      }
    }
  }
}

BT::NodeStatus BT::AsyncCoroActionNode::executeTick()
{
  // send signal to other thread.
  // The other thread is in charge for changing the status
  if (status() == BT::NodeStatus::IDLE)
  {
    setStatus(BT::NodeStatus::RUNNING);
    notifyStart();
  }

  if (status() == BT::NodeStatus::RUNNING)
  {
    notifyTick();
  }

  if (exptr_)
  {
    std::rethrow_exception(exptr_);
  }
  return status();
}

void BT::AsyncCoroActionNode::stopAndJoinThread()
{
  keep_thread_alive_.store(false);
  notifyStart();
  notifyTick();
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void BT::AsyncCoroActionNode::waitForTick()
{
  std::unique_lock<std::mutex> lLock(this->mTickMutex);
  this->mTickSignal.wait(lLock);
}

void BT::AsyncCoroActionNode::notifyTick()
{
  std::lock_guard<std::mutex>{this->mTickMutex};
  this->mTickSignal.notify_all();
}