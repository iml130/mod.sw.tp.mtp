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

#include "mars_agent_logical_agv/ROSNode.h"

static const std::string NODE_NAME = "mars_agent_logical_agv";

static const int NODE_RATE_MAX_ROS_PARAM = -1;
static const int NODE_RATE_MAX = 50;

static const std::string LOG_LEVEL_DEBUG = "debug";
static const std::string LOG_LEVEL_INFO = "info";
static const std::string LOG_LEVEL_WARN = "warn";
static const std::string LOG_LEVEL_ERROR = "error";
static const std::string LOG_LEVEL_FATAL = "fatal";

static const std::string STD_NODE_RATE_PARAM_NAME = "node_rate";
static const int STD_NODE_RATE_PARAM = 5;

static const std::string PARAM_NAME_CURRENT_TOPOLOGY_ENTITY_ID = "current_topology_entity_id";
static const std::string PARAM_NAME_CURRENT_TOPOLOGY_ENTITY_TYPE = "current_topology_entity_type";
static const std::string PARAM_NAME_PHYSICAL_AGENT_ID = "physical_agent_id";

static const std::string STD_LOG_LEVEL_PARAM_NAME = "log_level";
static const std::string STD_LOG_LEVEL_PARAM = LOG_LEVEL_INFO;

static const std::string PARAM_NAME_TOPOLOGY_NAMESPACE = "topology_namespace";
static const std::string PARAM_DEFAULT_TOPOLOGY_NAMESPACE = "/mars/topology/";

// Physical Agent topic names (publish)
static const std::string PARAM_NAME_TOPIC_NAME_CANCEL_ORDER = "topic_cancel_order";
static const std::string DEFAULT_TOPIC_NAME_CANCEL_ORDER = "cancel_order";
static const std::string PARAM_NAME_TOPIC_NAME_INIT_POSE = "topic_init_pose";
static const std::string DEFAULT_TOPIC_NAME_INIT_POSE = "init_pose";

// service names
static const std::string PARAM_NAME_SERVICE_NAME_MANUAL_TRIGGER = "service_manual_trigger";
static const std::string DEFAULT_SERVICE_NAME_MANUAL_TRIGGER = "manual_trigger";

template <typename T>
T mars::agent::logical::robot::ROSNode::getParam(ros::NodeHandle& nH, const std::string& paramName,
                                                 const T& defaultValue)
{
  T value;
  if (nH.getParam(paramName, value))
  {
    MARS_LOG_INFO("Found parameter: " << paramName << ", value: " << value);
    return value;
  }
  else
    MARS_LOG_INFO("Cannot find value for parameter: " << paramName
                                                      << ", assigning default: " << defaultValue);
  return defaultValue;
}

template <typename T>
void mars::agent::logical::robot::ROSNode::getParam(ros::NodeHandle& nH,
                                                    const std::string& paramName,
                                                    T& paramValue) noexcept(false)
{
  if (nH.getParam(paramName, paramValue))
  {
    MARS_LOG_INFO("Found parameter: " << paramName << ", value: " << paramValue);
  }
  else
  {
    throw mars::common::exception::ReadParamException("Could not read parameter: " + paramName);
  }
}

mars::agent::logical::robot::ROSNode::ROSNode() : mCurrentTopologyEntity(nullptr)
{
  this->mNHPriv = ros::NodeHandle("~");
  this->mLogicalAgentId.initialize();
  this->mCurrentPathId.initialize();
}

mars::agent::logical::robot::ROSNode::~ROSNode()
{
  if (this->mCurrentTopologyEntity != nullptr)
  {
    delete this->mCurrentTopologyEntity;
  }
}

bool mars::agent::logical::robot::ROSNode::init()
{
  bool initSuccessfully;

  // first read launch parameter
  initSuccessfully = this->readLaunchParams();

  // if no error occours...
  if (initSuccessfully)
  {
    // .. and continue with the initialization of the services
    initSuccessfully = this->initServices();
  }
  else
  {
    MARS_LOG_ERROR("Error while reading launch file params. Shutting down node!");
  }

  // ... if no error occours initialize the publisher
  if (initSuccessfully)
  {
    initSuccessfully = this->initPublisher();
  }
  else
  {
    MARS_LOG_ERROR("Error while initializing node Services "
                   "-> shutting down node!");
  }

  // ... if no error occours initialize the subscriber
  if (initSuccessfully)
  {
    initSuccessfully = this->initSubscriber();
  }
  else
  {
    MARS_LOG_ERROR("Error while setting up publisher -> "
                   "shutting down node!");
  }

  // ... if no error occours initialize the action server
  if (initSuccessfully)
  {
    initSuccessfully = this->initActions();
  }
  else
  {
    MARS_LOG_ERROR("Error while setting up subscriber -> "
                   "shutting down node!");
  }

  if (!initSuccessfully)
  {
    MARS_LOG_ERROR("Error while setting up action server -> "
                   "shutting down node!");
  }

  return initSuccessfully;
}

template <class T, class MReq, class MRes>
bool mars::agent::logical::robot::ROSNode::advertiseService(bool (T::*srvFunc)(MReq&, MRes&),
                                                            ros::ServiceServer& serviceServer,
                                                            std::string serviceName,
                                                            T* obj) noexcept(false)
{
  bool advertisedSuccessfully = true;

  try
  {
    serviceServer = this->mNHPriv.advertiseService(serviceName, srvFunc, obj);

    if (!serviceServer)
    {
      advertisedSuccessfully = false;
      throw mars::common::exception::AdvertiseServiceException("Could not advertise service: " +
                                                               serviceName);
    }
  }
  catch (const ros::InvalidNameException& e)
  {
    advertisedSuccessfully = false;
    MARS_LOG_ERROR("(InvalidNameException) "
                   "Exception occured: "
                   << e.what());
  }

  return advertisedSuccessfully;
}

bool mars::agent::logical::robot::ROSNode::initServices()
{
  bool initSuccessfully = true;

  try
  {
    this->mTriggerService = this->mNHPriv.advertiseService(
        this->mServiceNameManualTrigger,
        mars::agent::logical::common::behavior::WaitForManualTrigger::serviceTriggerCallback);
  }
  catch (mars::common::exception::AdvertiseServiceException& e)
  {
    initSuccessfully = false;
    MARS_LOG_ERROR("(AdvertisedServiceException) "
                   "Exception occured: "
                   << e.what());
  }

  return initSuccessfully;
}

bool mars::agent::logical::robot::ROSNode::initPublisher()
{
  bool initSuccessfully = true;

  this->mPublisherCancelTask = this->mNHPriv.advertise<mars_agent_physical_robot_msgs::CancelTask>(
      this->mTopicNameCancelTask, 1, true);
  this->mPublisherInitPose =
      this->mNHPriv.advertise<geometry_msgs::Pose>(this->mTopicNameInitPose, 1, true);

  return initSuccessfully;
}

bool mars::agent::logical::robot::ROSNode::initSubscriber()
{
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool mars::agent::logical::robot::ROSNode::initActions()
{
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool mars::agent::logical::robot::ROSNode::readLaunchParams()
{
  // IF a needed parameter is not provided, set 'initSuccessfully' to false!
  bool initSuccessfully = true;
  std::string lTopologyNamespace;

  // read optional params
  this->mNodeLogLevel = getParam(this->mNHPriv, STD_LOG_LEVEL_PARAM_NAME, STD_LOG_LEVEL_PARAM);
  // set node log level
  this->setNodeLogLevel();

  this->mHz = getParam(this->mNHPriv, STD_NODE_RATE_PARAM_NAME, STD_NODE_RATE_PARAM);
  this->mHz = (this->mHz == NODE_RATE_MAX_ROS_PARAM) ? NODE_RATE_MAX : this->mHz;

  lTopologyNamespace =
      getParam(this->mNHPriv, PARAM_NAME_TOPOLOGY_NAMESPACE, PARAM_DEFAULT_TOPOLOGY_NAMESPACE);

  mars::routing::common::topology::EdgeInterface::setNamespace(lTopologyNamespace);
  mars::routing::common::topology::VertexInterface::setNamespace(lTopologyNamespace);

  // read publish topics
  this->mTopicNameCancelTask =
      getParam(this->mNHPriv, PARAM_NAME_TOPIC_NAME_CANCEL_ORDER, DEFAULT_TOPIC_NAME_CANCEL_ORDER);
  this->mTopicNameInitPose =
      getParam(this->mNHPriv, PARAM_NAME_TOPIC_NAME_INIT_POSE, DEFAULT_TOPIC_NAME_INIT_POSE);

  // read service names
  this->mServiceNameManualTrigger = getParam(this->mNHPriv, PARAM_NAME_SERVICE_NAME_MANUAL_TRIGGER,
                                             DEFAULT_SERVICE_NAME_MANUAL_TRIGGER);

  try
  {
    std::string lPhysicalAgentIdString;
    std::string lCurrentTopologyEntityIdString;
    std::string lCurrentTopologyEntityTypeString;

    int lCurrentTopologyEntityType;

    this->getParam(this->mNHPriv, PARAM_NAME_PHYSICAL_AGENT_ID, lPhysicalAgentIdString);
    this->getParam(this->mNHPriv, PARAM_NAME_CURRENT_TOPOLOGY_ENTITY_ID,
                   lCurrentTopologyEntityIdString);
    this->getParam(this->mNHPriv, PARAM_NAME_CURRENT_TOPOLOGY_ENTITY_TYPE,
                   lCurrentTopologyEntityTypeString);

    this->mPhysicalAgentId = mars::common::Id(lPhysicalAgentIdString);
    mars::common::Id lCurrentTopologyEntityId(lCurrentTopologyEntityIdString);

    lCurrentTopologyEntityType = std::stoi(lCurrentTopologyEntityTypeString);

    mars::common::TimeInterval lReservationInterval(ros::Time::now(),
                                                    mars::common::TimeInterval::INFINITE_DURATION);

    if (lCurrentTopologyEntityType >=
            mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
        lCurrentTopologyEntityType <=
            mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
    {
      this->mCurrentTopologyEntity =
          new mars::routing::common::topology::Vertex(lCurrentTopologyEntityId);
      this->mCurrentTopologyEntity->addReservation(
          this->mPhysicalAgentId, this->mCurrentPathId, lReservationInterval);
      this->mCurrentTopologyEntity->allocate(this->mPhysicalAgentId,
                                             this->mCurrentPathId,
                                             lReservationInterval,
                                             ros::Duration(31536000, 0));

      MARS_LOG_INFO("Allocated starting vertex.");
    }
    else if (lCurrentTopologyEntityType >=
                 mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
             lCurrentTopologyEntityType <=
                 mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX)
    {
      this->mCurrentTopologyEntity =
          new mars::routing::common::topology::Edge(lCurrentTopologyEntityId);
      this->mCurrentTopologyEntity->addReservation(
          this->mPhysicalAgentId, this->mCurrentPathId, lReservationInterval);
      this->mCurrentTopologyEntity->allocate(this->mPhysicalAgentId,
                                             this->mCurrentPathId,
                                             lReservationInterval,
                                             ros::Duration(31536000, 0));

      MARS_LOG_INFO("Allocated starting edge.");
    }
    else
    {
      initSuccessfully = false;
      MARS_LOG_ERROR("Topology entity type is neither vertex nor edge!");
    }
  }
  catch (mars::common::exception::ReadParamException& e)
  {
    initSuccessfully = false;
    MARS_LOG_ERROR("(ReadParamException) Exception occured: " << e.what());
  }

  return initSuccessfully;
}

bool mars::agent::logical::robot::ROSNode::setNodeLogLevel() const
{
  ros::console::Level nodeLogLevel;
  bool setLogLevelSuccessfully = true;

  if (this->mNodeLogLevel == LOG_LEVEL_DEBUG)
  {
    nodeLogLevel = ros::console::levels::Debug;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_INFO)
  {
    nodeLogLevel = ros::console::levels::Info;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_WARN)
  {
    nodeLogLevel = ros::console::levels::Warn;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_ERROR)
  {
    nodeLogLevel = ros::console::levels::Error;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_FATAL)
  {
    nodeLogLevel = ros::console::levels::Fatal;
  }
  else
  {
    MARS_LOG_WARN("Wrong log level was "
                  "set in launch file! Level was '"
                  << this->mNodeLogLevel << "' but must be '" << LOG_LEVEL_DEBUG << "', '"
                  << LOG_LEVEL_INFO << "', '" << LOG_LEVEL_WARN << LOG_LEVEL_ERROR << "' or '"
                  << LOG_LEVEL_FATAL);

    setLogLevelSuccessfully = false;
  }

  // set node log level
  if (setLogLevelSuccessfully &&
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, nodeLogLevel))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  else
  {
    MARS_LOG_WARN("Can not set ros logger level with level: " << this->mNodeLogLevel);

    setLogLevelSuccessfully = false;
  }

  return setLogLevelSuccessfully;
}

void mars::agent::logical::robot::ROSNode::rosMainLoop() const
{
  ros::Rate rate(this->mHz);
  MARS_LOG_INFO("Node successfully initialized. Starting node now!");

  BT::BehaviorTreeFactory lBehaviorTreeFactory = this->registerBehavior();
  BT::Tree lTree = lBehaviorTreeFactory.createTreeFromFile(
      ros::package::getPath("mars_agent_logical_agv") + "/behavior/LogicalAgentReconstruct.xml");

  // BT::PublisherZMQ logger(lTree);

  // BT::StdCoutLogger lCoutLogger(lTree);

  // BT::CoutDurationLogger lCoutDurationLogger(lTree);

  while (ros::ok())
  {
    ros::spinOnce();
    lTree.rootNode()->executeTick();
    rate.sleep();
  }

  // Stop the node's resources
  ros::shutdown();
}

BT::BehaviorTreeFactory mars::agent::logical::robot::ROSNode::registerBehavior() const
{
  BT::BehaviorTreeFactory lBehaviorTreeFactory;
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::ExtractMoveOrder>(
      "ExtractMoveOrder");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::ImportROSParameter>(
      "ImportROSParameter");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::PlanRoute>(
      "PlanRoute");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetCurrentMotion>(
      "SetCurrentMotion");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetCurrentOrder>(
      "SetCurrentOrder");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetCurrentTime>(
      "SetCurrentTime");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetId>("SetId");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetRobotProperties>(
      "SetRobotProperties");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetTopologyEntity>(
      "SetTopologyEntity");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::AllocateEntity>(
      "AllocateEntity");
  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::DeallocateEntity>(
      "DeallocateEntity");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::PublishOrderStatus>(
      "PublishOrderStatus");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::RepeatOncePerTick>(
      "RepeatOncePerTick");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::RepeatUntilDone>(
      "RepeatUntilDone");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::IsMoveOrder>(
      "IsMoveOrder");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::IsManualAction>(
      "IsManualAction");

  lBehaviorTreeFactory
      .registerNodeType<mars::agent::logical::common::behavior::AreTopologyEntitiesEqual>(
          "AreTopologyEntitiesEqual");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::AreIdsEqual>(
      "AreIdsEqual");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::IsTransportOrder>(
      "IsTransportOrder");

  lBehaviorTreeFactory
      .registerNodeType<mars::agent::logical::common::behavior::ExtractTransportOrder>(
          "ExtractTransportOrder");

  lBehaviorTreeFactory
      .registerNodeType<mars::agent::logical::common::behavior::ExtractTransportOrderStep>(
          "ExtractTransportOrderStep");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetOrderStatus>(
      "SetOrderStatus");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetOrderState>(
      "SetOrderState");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetOrderType>(
      "SetOrderType");

  lBehaviorTreeFactory
      .registerNodeType<mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget>(
          "PublishMotionAssignmentsToTarget");

  lBehaviorTreeFactory
      .registerNodeType<mars::agent::logical::common::behavior::WaitForManualTrigger>(
          "WaitForManualTrigger");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::WaitForNSeconds>(
      "WaitForNSeconds");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::SetDuration>(
      "SetDuration");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::IsTrue>("IsTrue");

  lBehaviorTreeFactory
      .registerNodeType<mars::agent::logical::common::behavior::RepeatUntilSuccessful>(
          "RepeatUntilSuccessful");

  lBehaviorTreeFactory.registerNodeType<mars::agent::logical::common::behavior::StringToBool>(
      "StringToBool");

  return lBehaviorTreeFactory;
}

bool mars::agent::logical::robot::ROSNode::run()
{
  bool runNodeSuccessfully = true;

  if (this->init())
  {
    this->rosMainLoop();
  }
  else
  {
    runNodeSuccessfully = false;
  }

  return runNodeSuccessfully;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);

  mars::agent::logical::robot::ROSNode marsAgentLogicalRobot;

  return marsAgentLogicalRobot.run() ? EXIT_SUCCESS : EXIT_FAILURE;
}
