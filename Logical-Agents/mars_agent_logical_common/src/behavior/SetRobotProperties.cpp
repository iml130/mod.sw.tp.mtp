#include "mars_agent_logical_common/behavior/SetRobotProperties.h"
#include "mars_common/Logger.h"

BT::NodeStatus mars::agent::logical::common::behavior::SetRobotProperties::tick()
{
  BT::Result lResult;

  if (!this->mRobotPropertiesSubscriber)
  {
    BT::Optional<std::string> lTopicNameRobotProperties;

    lTopicNameRobotProperties = this->getInput<std::string>(
        BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_TOPIC_NAME_ROBOT_PROPERTIES);
    if (!lTopicNameRobotProperties)
    {
      MARS_LOG_WARN(lTopicNameRobotProperties.error());
      lTopicNameRobotProperties.value() =
          BEHAVIOR_SETCURRENTMOTION_DEFAULT_TOPIC_NAME_ROBOT_PROPERTIES;
    }

    this->mRobotPropertiesSubscriber =
        this->mNodeHandle.subscribe<mars_agent_physical_robot_msgs::RobotAgentProperties>(
            lTopicNameRobotProperties.value(), 10,
            &mars::agent::logical::common::behavior::SetRobotProperties::
                subscriberCallbackRobotProperties,
            this);
  }

  std::unique_lock<std::mutex> lLock(this->mRobotPropertiesMutex);
  while (!this->mRobotProperties)
  {
    lLock.unlock();
    this->setStatusRunningAndYield();
    lLock.lock();
  }
  lLock.unlock();

  MARS_LOG_INFO("Received robot agent properties from physical agent (RAN).");
  MARS_LOG_INFO("Logical Agent is now successfully initialized");

  std::lock_guard<std::mutex>{this->mRobotPropertiesMutex};

  lResult = this->setOutput<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
      BEHAVIOR_SETCURRENTMOTION_PARAM_NAME_ROBOT_PROPERTIES,
      std::make_shared<mars::agent::physical::common::RobotAgentProperties>(
          *this->mRobotProperties));
  if (!lResult)
  {
    MARS_LOG_ERROR(lResult.error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void mars::agent::logical::common::behavior::SetRobotProperties::subscriberCallbackRobotProperties(
    const mars_agent_physical_robot_msgs::RobotAgentProperties::ConstPtr& pRobotProperties)
{
  std::lock_guard<std::mutex>{this->mRobotPropertiesMutex};

  this->mRobotProperties = mars::agent::physical::common::RobotAgentProperties(*pRobotProperties);
}
