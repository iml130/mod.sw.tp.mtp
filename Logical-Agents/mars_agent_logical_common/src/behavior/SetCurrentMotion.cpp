#include "mars_agent_logical_common/behavior/SetCurrentMotion.h"
#include "mars_agent_logical_common/MoveOrder.h"
#include "mars_common/Logger.h"

#include <eigen_conversions/eigen_msg.h>

mars::agent::logical::common::behavior::SetCurrentMotion::SetCurrentMotion(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::CoroActionNode(pName, pConfig)
{
}

BT::NodeStatus mars::agent::logical::common::behavior::SetCurrentMotion::tick()
{
  BT::Result lResult;
  Eigen::Affine3d lPose;
  Eigen::Matrix<double, 6, 1> lTwist;
  mars_agent_physical_robot_msgs::Motion lCurrentMotion;

  if (!this->mMotionSubscriber)
  {
    BT::Optional<std::string> lTopicNameMotion;

    lTopicNameMotion =
        this->getInput<std::string>(BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TOPIC_NAME_MOTION);
    if (!lTopicNameMotion)
    {
      MARS_LOG_WARN(lTopicNameMotion.error());
      lTopicNameMotion.value() = BEHAVIOR_SETCURRENTMOTION_DEFAULT_TOPIC_NAME_MOTION;
    }

    this->mMotionSubscriber = this->mNodeHandle.subscribe<mars_agent_physical_robot_msgs::Motion>(
        lTopicNameMotion.value(), 100,
        &mars::agent::logical::common::behavior::SetCurrentMotion::subscriberCallbackCurrentMotion,
        this);
  }

  std::unique_lock<std::mutex> lLock(this->mMotionMutex);
  while (this->mCurrentMotionBuffer.empty())
  {
    lLock.unlock();
    this->setStatusRunningAndYield();
    lLock.lock();
  }
  lLock.unlock();

  lCurrentMotion = this->mCurrentMotionBuffer.front();
  this->mCurrentMotionBuffer.pop_front();

  // TODO: TF lookup and transforming local frame to world frame, so this->mCurrentPose stores
  // absolute pose of the robot in the world frame.
  tf::poseMsgToEigen(lCurrentMotion.current_position.pose, lPose);
  tf::twistMsgToEigen(lCurrentMotion.current_velocity, lTwist);

  this->mPose = lPose;
  this->mTwist = lTwist;

  std::lock_guard<std::mutex>{this->mMotionMutex};

  lResult =
      this->setOutput<Eigen::Affine3d>(BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_POSE, *this->mPose);
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  lResult = this->setOutput<Eigen::Matrix<double, 6, 1>>(BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TWIST,
                                                         *this->mTwist);
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void mars::agent::logical::common::behavior::SetCurrentMotion::subscriberCallbackCurrentMotion(
    const mars_agent_physical_robot_msgs::Motion::ConstPtr& pMotionMsg)
{
  std::lock_guard<std::mutex>{this->mMotionMutex};

  this->mCurrentMotionBuffer.push_back(*pMotionMsg);
}
