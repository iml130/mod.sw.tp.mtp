/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : mars_yellow_pages
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * --------------------------------------------------------------------------------------------- */

// TODO: reduce CMakeLists.txt to a minimum, add all needed libs and keep it
//       clean and up to date!
// TODO: create a launch file with following parameter inside: 'node_rate', 'log_level'

#include "mars_yellow_pages/ROSNode.h"

#include <mars_common/Id.h>
#include <mars_common/Logger.h>
#include <mars_common/exception/SetParamException.h>

static const std::string NODE_NAME = "mars_yellow_pages";

static const std::string LOG_LEVEL_DEBUG = "debug";
static const std::string LOG_LEVEL_INFO = "info";
static const std::string LOG_LEVEL_WARN = "warn";
static const std::string LOG_LEVEL_ERROR = "error";
static const std::string LOG_LEVEL_FATAL = "fatal";

static const std::string STD_LOG_LEVEL_PARAM_NAME = "log_level";
static const std::string STD_LOG_LEVEL_PARAM = LOG_LEVEL_INFO;

// Standard service name to find the container id for a topology entity
static const std::string STD_SERVICE_NAME_LOOKUP_CONTAINER = "lookup_container";

// Standard topic name to register all topology entities of a container
static const std::string STD_TOPIC_NAME_REGISTER_CONTAINER = "register_container";

static const std::string STD_RESULT_MSG_ADDED_ID_SUCCESSFUL = "Added container ID successful: ";
static const std::string STD_RESULT_MSG_ADDED_ID_UNSUCCESSFUL =
    "Container ID for entity cannot be found!";

template <typename T>
T ROSNode::getParam(ros::NodeHandle& nH, const std::string& paramName, const T& defaultValue)
{
  T value;
  if (nH.getParam(paramName, value))
  {
    ROS_INFO_STREAM("Found parameter: " << paramName << ", value: " << value);
    return value;
  }
  else
    ROS_INFO_STREAM("Cannot find value for parameter: " << paramName
                                                        << ", assigning default: " << defaultValue);
  return defaultValue;
}

template <typename T>
void ROSNode::getParam(ros::NodeHandle& nH, const std::string& paramName, T& paramValue)
{

  if ((nH.getParam(paramName, paramValue)))
  {
    ROS_INFO_STREAM("Found parameter: " << paramName << ", value: " << paramValue);
  }
  else
  {
    throw mars::common::exception::ReadParamException("Could not read parameter: " + paramName);
  }
}

template <class T, class MReq, class MRes>
bool ROSNode::advertiseService(bool (T::*srvFunc)(MReq&, MRes&), ros::ServiceServer& serviceServer,
                               std::string serviceName, T* obj)
{
  bool advertisedSuccessfully = true;

  try
  {
    serviceServer = this->mNH.advertiseService(serviceName, srvFunc, obj);
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
    ROS_ERROR_STREAM("[MarsTopologyEdge::initServices](InvalidNameException) "
                     "Exception occured: "
                     << e.what());
  }

  return advertisedSuccessfully;
}

ROSNode::ROSNode() { this->mNHPriv = ros::NodeHandle("~"); }

ROSNode::~ROSNode() {}

void ROSNode::subscriberCallbackRegisterContainer(
    const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& msg)
{
  this->mMARSYellowPages.addContainer(msg);
}

bool ROSNode::serviceCallbackLookupContainer(
    mars_topology_srvs::GetTopologyEntityContainerId::Request& req,
    mars_topology_srvs::GetTopologyEntityContainerId::Response& res)
{
  try
  {
    mars::common::Id lContainerId = this->mMARSYellowPages.getContainerId(req);

    res.container_id = mars::common::Id::convertToMsgId(lContainerId);

    if (lContainerId.isValid())
    {
      res.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
      res.result.description = STD_RESULT_MSG_ADDED_ID_SUCCESSFUL + lContainerId.getUUIDAsString(mars::common::Id::HEXDEC_SPLIT);
    }
    else
    {
      res.result.result = mars_common_msgs::Result::RESULT_ERROR;
      res.result.description = STD_RESULT_MSG_ADDED_ID_UNSUCCESSFUL;
    }
  }
  catch (const mars::common::exception::SetParamException& e)
  {
    MARS_LOG_ERROR("Error while proccesing Id from request: " << e.what());
  }

  return true;
}

bool ROSNode::init()
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
    ROS_ERROR("[ROSNode::init] Error while reading launch file params "
              "-> shutting down node!");
  }

  // ... if no error occours initialize the publisher
  if (initSuccessfully)
  {
    initSuccessfully = this->initPublisher();
  }
  else
  {
    ROS_ERROR("[ROSNode::init] Error while initializing node Services "
              "-> shutting down node!");
  }

  // ... if no error occours initialize the subscriber
  if (initSuccessfully)
  {
    initSuccessfully = this->initSubscriber();
  }
  else
  {
    ROS_ERROR("[ROSNode::init] Error while setting up publisher -> "
              "shutting down node!");
  }

  // ... if no error occours initialize the action server
  if (initSuccessfully)
  {
    initSuccessfully = this->initActions();
  }
  else
  {
    ROS_ERROR("[ROSNode::init] Error while setting up subscriber -> "
              "shutting down node!");
  }

  if (!initSuccessfully)
  {
    ROS_ERROR("[ROSNode::init] Error while setting up action server -> "
              "shutting down node!");
  }

  return initSuccessfully;
}

bool ROSNode::initServices()
{
  bool initSuccessfully = true;

  this->advertiseService(&ROSNode::serviceCallbackLookupContainer, this->mServiceLookupContainer,
                         STD_SERVICE_NAME_LOOKUP_CONTAINER, this);

  return initSuccessfully;
}

bool ROSNode::initPublisher()
{
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool ROSNode::initSubscriber()
{
  bool initSuccessfully = true;

  // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  this->mSubscriberRegisterContainer = this->mNH.subscribe(
      STD_TOPIC_NAME_REGISTER_CONTAINER, 100, &ROSNode::subscriberCallbackRegisterContainer, this);

  return initSuccessfully;
}

bool ROSNode::initActions()
{
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool ROSNode::readLaunchParams()
{
  // IF a needed parameter is not provided, set 'initSuccessfully' to false!
  bool initSuccessfully = true;

  // read program params
  {
    this->mNodeLogLevel = getParam(this->mNHPriv, STD_LOG_LEVEL_PARAM_NAME, STD_LOG_LEVEL_PARAM);
    // set node log level
    this->setNodeLogLevel();
  }

  return initSuccessfully;
}

bool ROSNode::setNodeLogLevel() const
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
    ROS_WARN_STREAM("[ROSNode::setNodeLogLevel] Wrong log level was "
                    "set in launch file! Level was '"
                    << this->mNodeLogLevel << "' but must be '" << LOG_LEVEL_DEBUG << "', '"
                    << LOG_LEVEL_INFO << "', '" << LOG_LEVEL_WARN << "', '" << LOG_LEVEL_ERROR
                    << "' or '" << LOG_LEVEL_FATAL << "'.");
    ;

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
    ROS_WARN_STREAM("[ROSNode::setNodeLogLevel] Can not set ros logger "
                    "level with level: "
                    << this->mNodeLogLevel);

    setLogLevelSuccessfully = false;
  }

  return setLogLevelSuccessfully;
}

void ROSNode::rosMainLoop() const
{
  ROS_INFO("[ROSNode::rosMainLoop] Node successfully initialized. Starting node now!");

  ros::spin();

  // Stop the node's resources
  ros::shutdown();
}

bool ROSNode::runROSNode()
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

  ROSNode lROSNode;

  return lROSNode.runROSNode() ? EXIT_SUCCESS : EXIT_FAILURE;
}
