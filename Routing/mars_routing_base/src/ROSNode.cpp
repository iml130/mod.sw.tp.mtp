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


#include "mars_routing_base/ROSNode.h"

#include "mars_routing_common/topology/EdgeInterface.h"
#include "mars_routing_common/topology/VertexInterface.h"

#include <visualization_msgs/MarkerArray.h>

static const std::string NODE_NAME = "mars_routing_base_router";

static const int NODE_RATE_MAX_ROS_PARAM = -1;
static const int NODE_RATE_MAX = 50;

static const double VISUALIZATION_RATE = 5;

static const std::string LOG_LEVEL_DEBUG = "debug";
static const std::string LOG_LEVEL_INFO = "info";
static const std::string LOG_LEVEL_WARN = "warn";
static const std::string LOG_LEVEL_ERROR = "error";
static const std::string LOG_LEVEL_FATAL = "fatal";

static const std::string PARAM_NAME_NODE_RATE = "node_rate";
static const int PARAM_DEFAULT_NODE_RATE = 5;

static const std::string PARAM_NAME_LOG_LEVEL = "log_level";
static const std::string PARAM_DEFAULT_LOG_LEVEL = LOG_LEVEL_DEBUG;

static const std::string PARAM_NAME_PLUGIN_NAME = "plugin_name";
static const std::string PARAM_DEFAULT_PLUGIN_NAME = "marsRoutingPlugin/Carp";

static const std::string PARAM_NAME_TOPOLOGY_NAMESPACE = "topology_namespace";
static const std::string PARAM_DEFAULT_TOPOLOGY_NAMESPACE = "/mars/topology/";

static const std::string PARAM_NAME_VISUALIZATION = "route_visualization";
static const bool PARAM_DEFAULT_VISUALIZATION = false;

static const std::string SET_VISUALIZATION_SERVICE = "setRouteVisualization";
static const std::string CLEAR_VISUALIZATION_SERVICE = "clearRouteVisualization";
static const std::string REMOVE_VISUALIZATION_SERVICE = "removeRouteVisualizations";
static const std::string VISUALIZATION_ROUTE_TOPIC = "visualizationRouteRaw";
static const std::string SET_VISUALIZATION_START_TIME_SERVICE = "setVisualizationStartTime";

template <typename T>
T getParam(ros::NodeHandle& nH, const std::string& name, const T& defaultValue)
{
  T value;
  if (nH.getParam(name, value))
  {
    ROS_DEBUG_STREAM("Found parameter: " << name << ", value: " << value);
    return value;
  }
  else
    ROS_DEBUG_STREAM(
        "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  return defaultValue;
}

mars::routing::base::ROSNode::ROSNode()
    : mRouterLoader("mars_routing_core", "mars::routing::core::Router")
{
  this->mNHPriv = ros::NodeHandle("~");
}

mars::routing::base::ROSNode::~ROSNode() {}

bool mars::routing::base::ROSNode::init()
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
    ROS_ERROR("[mars::routing::base::ROSNode::init] Error while reading launch file params "
              "-> shutting down node!");
  }

  // ... if no error occours initialize the publisher
  if (initSuccessfully)
  {
    initSuccessfully = this->initPublisher();
  }
  else
  {
    ROS_ERROR("[mars::routing::base::ROSNode::init] Error while initializing node Services "
              "-> shutting down node!");
  }

  // ... if no error occours initialize the subscriber
  if (initSuccessfully)
  {
    initSuccessfully = this->initSubscriber();
  }
  else
  {
    ROS_ERROR("[mars::routing::base::ROSNode::init] Error while setting up publisher -> "
              "shutting down node!");
  }

  // ... if no error occours initialize the action server
  if (initSuccessfully)
  {
    initSuccessfully = this->initActions();
  }
  else
  {
    ROS_ERROR("[mars::routing::base::ROSNode::init] Error while setting up subscriber -> "
              "shutting down node!");
  }

  if (initSuccessfully)
  {
    initSuccessfully = this->loadRouterPlugin();
  }
  else
  {
    ROS_ERROR("[mars::routing::base::ROSNode::init] Error while setting up action server -> "
              "shutting down node!");
  }

  if (!initSuccessfully)
  {
    ROS_ERROR("[mars::routing::base::ROSNode::init] Error while loading the Router Plugin -> "
              "shutting down node!");
  }

  mRoutes.resize(0);
  mVisualizationStartTime = ros::Time(0,0);
  if (mRunVisualization)
  {
    mVisualizationThread =
        std::make_shared<std::thread>(&mars::routing::base::ROSNode::visualizationLoop, this);
  }
  return initSuccessfully;
}

bool mars::routing::base::ROSNode::initServices()
{
  bool initSuccessfully = false;

  mServiceGetRoute =
      mNHPriv.advertiseService("GetRoute", &mars::routing::base::ROSNode::getRoute, this);

  mServiceRemoveVisualizedRoutes = mNHPriv.advertiseService(
      REMOVE_VISUALIZATION_SERVICE, &mars::routing::base::ROSNode::removeRouteVisualization, this);

  mServiceSetVisualizationStartTime =
      mNHPriv.advertiseService(SET_VISUALIZATION_START_TIME_SERVICE,
                               &mars::routing::base::ROSNode::setRouteVisualizationStartTime, this);

  // Enter if Service is valid
  if (mServiceGetRoute)
  {
    initSuccessfully = true;
  }

  return initSuccessfully;
}

bool mars::routing::base::ROSNode::initPublisher()
{
  this->mVisualizationPub =
      this->mNH.advertise<visualization_msgs::MarkerArray>(VISUALIZATION_ROUTE_TOPIC, 10);
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool mars::routing::base::ROSNode::initSubscriber()
{
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool mars::routing::base::ROSNode::initActions()
{
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool mars::routing::base::ROSNode::readLaunchParams()
{
  // IF a needed parameter is not provided, set 'initSuccessfully' to false!
  bool initSuccessfully = true;
  std::string lTopologyNamespace;

  // read program params
  {
    this->mNodeLogLevel = getParam(this->mNHPriv, PARAM_NAME_LOG_LEVEL, PARAM_DEFAULT_LOG_LEVEL);
    // set node log level
    this->setNodeLogLevel();
  }

  this->mHz = getParam(this->mNHPriv, PARAM_NAME_NODE_RATE, PARAM_DEFAULT_NODE_RATE);
  this->mHz = (this->mHz == NODE_RATE_MAX_ROS_PARAM) ? NODE_RATE_MAX : this->mHz;

  this->mPluginName = getParam(this->mNHPriv, PARAM_NAME_PLUGIN_NAME, PARAM_DEFAULT_PLUGIN_NAME);
  lTopologyNamespace =
      getParam(this->mNHPriv, PARAM_NAME_TOPOLOGY_NAMESPACE, PARAM_DEFAULT_TOPOLOGY_NAMESPACE);
  this->mRunVisualization =
      getParam(this->mNHPriv, PARAM_NAME_VISUALIZATION, PARAM_DEFAULT_VISUALIZATION);

  mars::routing::common::topology::EdgeInterface::setNamespace(lTopologyNamespace);
  mars::routing::common::topology::VertexInterface::setNamespace(lTopologyNamespace);

  return initSuccessfully;
}

bool mars::routing::base::ROSNode::setNodeLogLevel() const
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
    ROS_WARN_STREAM("[mars::routing::base::ROSNode::setNodeLogLevel] Wrong log level was "
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
    ROS_WARN_STREAM("[mars::routing::base::ROSNode::setNodeLogLevel] Can not set ros logger "
                    "level with level: "
                    << this->mNodeLogLevel);

    setLogLevelSuccessfully = false;
  }

  return setLogLevelSuccessfully;
}

void mars::routing::base::ROSNode::rosMainLoop() const
{
  ros::Rate rate(this->mHz);
  ROS_INFO("[mars::routing::base::ROSNode::rosMainLoop] Node successfully initialized. "
           "Starting node now!");

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   rate.sleep();
  ros::spin();
  // }

  // Stop the node's resources
  ros::shutdown();
}

bool mars::routing::base::ROSNode::run()
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

bool mars::routing::base::ROSNode::loadRouterPlugin()
{
  ros::console::Level nodeLogLevel;
  try
  {
    mRouterAlg = mRouterLoader.createInstance(this->mPluginName);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("[mars::routing::base::ROSNode::loadRouterPlugin] The plugin failed to "
              "load for some reason. Error: %s ->"
              "shutting down node!",
              ex.what());
    return false;
  }

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

  // Call the init function of the Plugin
  this->mRouterAlg->initialize(nodeLogLevel);

  ROS_INFO("[mars::routing::base::ROSNode::loadRouterPlugin] Loading of the Plugin %s "
           "was successful",
           this->mPluginName.c_str());

  return true;
}

bool mars::routing::base::ROSNode::getRoute(mars_routing_srvs::GetRoute::Request& req,
                                            mars_routing_srvs::GetRoute::Response& res)
{
  bool result = mRouterAlg->getRoute(req, res);

  if (res.result.result != res.result.RESULT_ERROR &&
      req.origin.entity_type.entity_type >=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      req.origin.entity_type.entity_type <=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX &&
      req.destination.entity_type.entity_type >=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      req.destination.entity_type.entity_type <=
          mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
  {
    auto* lRoutePtr = new mars::routing::core::Route<mars::routing::common::topology::Vertex,
                                                     mars::routing::common::topology::Edge,
                                                     mars::routing::common::topology::Vertex>(
        mars::routing::core::Route<mars::routing::common::topology::Vertex,
                                   mars::routing::common::topology::Edge,
                                   mars::routing::common::topology::Vertex>(
            mars_routing_srvs::GetRoute{req, res}));
    this->mVisualizationMutex.lock();
    this->mRoutes.push_back(lRoutePtr);
    this->mVisualizationMutex.unlock();
  }

  return result;
}

bool mars::routing::base::ROSNode::removeRouteVisualization(
    mars_routing_srvs::RemoveRouteVisualization::Request& req,
    mars_routing_srvs::RemoveRouteVisualization::Response& res)
{
  MARS_LOG_WARN("NYI");
  return true;
}

bool mars::routing::base::ROSNode::setRouteVisualizationStartTime(
    mars_routing_srvs::SetRouteVisualizationStartTime::Request& req,
    mars_routing_srvs::SetRouteVisualizationStartTime::Response& res)
{
  mVisualizationStartTime = req.start_time;
  return true;
}

void mars::routing::base::ROSNode::visualizationLoop()
{
  this->mRunVisualization = true;
  ros::Rate lRate(VISUALIZATION_RATE);

  // auto pointMsg = [](Eigen::Vector3d pVec, double pHeightParam) {
  //   geometry_msgs::Point lPoint;
  //   lPoint.x = pVec[0];
  //   lPoint.y = pVec[1];
  //   lPoint.z = 0.3 + 20.0 * std::min(1.0, std::max(0.0, pHeightParam));
  //   return lPoint;
  // };

  while (ros::ok() && this->mRunVisualization)
  {

    visualization_msgs::MarkerArray lMarkers;
    lMarkers.markers.reserve(this->mRoutes.size());

    this->mVisualizationMutex.lock();
    for (auto iRoute : this->mRoutes)
    {
      lMarkers.markers.push_back(iRoute->visualize(mVisualizationStartTime));
    }
    this->mVisualizationMutex.unlock();

    this->mVisualizationPub.publish(lMarkers);
    lRate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);

  mars::routing::base::ROSNode node;

  return node.run() ? EXIT_SUCCESS : EXIT_FAILURE;
}
